#!/usr/bin/env python
import os
import re
import sys
import logging
import socket
from queue import Queue

from e3.main import Main
from e3.fs import find, rm, mkdir, cp, ls
from e3.os.fs import which
from e3.collection.dag import DAG
from e3.job.walk import Walk
from e3.job import ProcessJob
from e3.env import Env
from e3.os.process import Run

# Directory in which the run-tests script is located
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))

# Directory in which tests are found
TEST_DIR = os.path.join(ROOT_DIR, 'tests')

# Result dir
RESULT_DIR = os.path.join(ROOT_DIR, 'results')

sys.path.insert(0, ROOT_DIR)


def service_name_to_source_pattern(service_name: str) -> str:
    """Convert a test service name to a source file pattern.

    :param service_name: the test service name (e.g., 'sensor-manager', 'arv')
    :return: the source file pattern to match (e.g., 'SensorManagerService', 'AutomationRequestValidatorService')
    """
    # Mapping of known test service names to source file patterns
    service_mapping = {
        'arv': 'AutomationRequestValidatorService',
        'sensor-manager': 'SensorManagerService',
        'assignment-tree': 'AssignmentTreeBranchBound',
        'automation-diagram': 'AutomationDiagramDataService',
        'batch-summary': 'BatchSummaryService',
    }

    return service_mapping.get(service_name, None)


def dump_gcov_summary(source_dir: str,
                      build_dir: str,
                      gcda_dir: str,
                      display_includes_coverage: bool,
                      service_filter: str = None) -> None:
    """Display a coverage summary.

    :param source_dir: root source dir
    :param gcda_dir: directory containing the gcda files
    :param display_includes_coverage: if True display coverage summary for
        include files
    :param service_filter: optional service name to filter coverage results
    :param gcda_files: a list of gcda files to process with gcov
    :param source_files: a list of source files to consider. Coverage
        information about files not in this list will not be displayed.
    """

    # Reset the directory containing the gcov files
    gcr = os.path.join(RESULT_DIR, 'gcov')
    rm(gcr, recursive=True)
    mkdir(gcr)

    # Run gcov to produce de gcov files
    for f in find(root=build_dir, pattern='*.gcno'):
        target_path = os.path.join(gcda_dir, os.path.relpath(f, build_dir))
        # Ensure parent directory exists before copying
        mkdir(os.path.dirname(target_path), quiet=True)
        cp(f, target_path)

    gcno_files = find(root=gcda_dir, pattern='*.gcno')

    gcov_out = os.path.join(gcr, 'gcov.out')

    # Run gcov from source_dir so it can resolve relative source paths
    # (e.g. "src/cpp/..." embedded in the gcno files).  The *.gcov output
    # files land in source_dir and are moved to gcr afterwards.
    # ENABLE_GCOV=1 is required when the GNAT gcov wrapper is on the PATH.
    Run(['gcov', '-p'] + gcno_files, cwd=source_dir,
        env={'ENABLE_GCOV': '1'},
        output=gcov_out)

    # Detect gcov version mismatch: if the gcno/gcda files were produced by a
    # different GCC than the gcov on the PATH, gcov emits warnings of the form
    # "version 'B52 ', prefer 'B33*'" and generates only stub output files.
    version_mismatch = re.compile(r"version '[^']+', prefer ")
    with open(gcov_out) as fd:
        for line in fd:
            if version_mismatch.search(line):
                # Clean up any stub files left in source_dir before exiting.
                for f in ls(os.path.join(source_dir, '*.gcov')):
                    rm(f)
                gcov_path = which('gcov') or 'gcov'
                logging.critical(
                    "gcov version mismatch detected.\n"
                    "  gcov in use : %s\n"
                    "  gcov output : %s\n\n"
                    "The gcov resolved from your PATH (%s) does not match the "
                    "GCC version used to build OpenUxAS with coverage "
                    "instrumentation.  Please ensure that you launch this "
                    "script with the same compiler environment that was used "
                    "during the build (e.g. run 'ensure_gnat' or add the "
                    "correct toolchain to your PATH before running run-tests).",
                    gcov_path, gcov_out, gcov_path)
                raise SystemExit(1)

    # Move gcov output files from source_dir to gcr.  Using an explicit loop
    # over ls() is safe when no files match (unlike a shell mv glob).
    for f in ls(os.path.join(source_dir, '*.gcov')):
        cp(f, gcr)
        rm(f)

    # Determine the source pattern to filter by
    source_pattern = None
    if service_filter:
        source_pattern = service_name_to_source_pattern(service_filter)

    total_sources = 0
    total_covered = 0
    file_count = 0

    for gcov_file in ls(os.path.join(gcr, '*.gcov')):
        # Decode original source paths (-p option of gcov)
        source_file = os.path.basename(gcov_file).replace('#', '/')[:-5]

        # Ignores all source that are not part of the project
        if os.path.isabs(source_file):
            continue

        # Filter by service pattern if specified
        if source_pattern:
            # Extract the base filename without extension
            base_name = os.path.basename(source_file)
            # Remove extension (.cpp, .h, .hpp)
            base_name_no_ext = os.path.splitext(base_name)[0]
            # Check if it matches the service pattern
            if not base_name_no_ext.startswith(source_pattern):
                continue

        if not display_includes_coverage and \
                (source_file.endswith('.h') or source_file.endswith('.hpp')):
            continue

        with open(gcov_file) as fd:
            total = 0
            covered = 0
            for line in fd:
                if re.match(r' *-:', line):
                    pass
                elif re.match(r' *[#=]{5}:', line):
                    total += 1
                else:
                    total += 1
                    covered += 1

        # Update global counters
        total_sources += total
        total_covered += covered
        file_count += 1

        # Display file information
        if total == 0:
            percent = 0.0
        else:
            percent = float(covered) * 100.0 / float(total)

        logging.info('%6.2f %% %8d/%-8d %s',
                     percent,
                     covered,
                     total,
                     source_file)

    # Display global counters (only if multiple files were processed)
    if file_count > 1:
        if total_sources == 0:
            percent = 0.0
        else:
            percent = float(total_covered) * 100.0 / float(total_sources)

        logging.info('%6.2f %% %8d/%-8d %s',
                     percent,
                     total_covered,
                     total_sources,
                     'TOTAL')


class TestJob(ProcessJob):
    """Handle a test execution."""

    PORTS: Queue = Queue()

    # Set to True when --impl=both so that 4 ports are reserved per job.
    BACK_TO_BACK: bool = False

    @property
    def cmdline(self):
        """See e3.job.ProcessJob."""
        return [sys.executable, self.data.test_path]

    def on_start(self, scheduler):
        logging.info("[%-10s %-9s %4ds] %s",
                     self.queue_name, 'start', 0, self.data)
        self.in_port = self.PORTS.get()
        self.out_port = self.PORTS.get()
        if TestJob.BACK_TO_BACK:
            self.challenger_in_port = self.PORTS.get()
            self.challenger_out_port = self.PORTS.get()
            logging.debug('Reserve ports: %s, %s, %s, %s',
                          self.in_port, self.out_port,
                          self.challenger_in_port, self.challenger_out_port)
        else:
            self.challenger_in_port = None
            self.challenger_out_port = None
            logging.debug('Reserve ports: %s, %s', self.in_port, self.out_port)

    def on_finish(self, scheduler):
        self.PORTS.put(self.in_port)
        self.PORTS.put(self.out_port)
        if TestJob.BACK_TO_BACK:
            self.PORTS.put(self.challenger_in_port)
            self.PORTS.put(self.challenger_out_port)
            logging.debug('Release ports: %s, %s, %s, %s',
                          self.in_port, self.out_port,
                          self.challenger_in_port, self.challenger_out_port)
        else:
            logging.debug('Release ports: %s, %s', self.in_port, self.out_port)

    @property
    def cmd_options(self):
        """See e3.job.ProcessJob."""
        env = {
            'IN_SERVER_URL': 'tcp://127.0.0.1:%s' % self.in_port,
            'OUT_SERVER_URL': 'tcp://127.0.0.1:%s' % self.out_port,
        }
        if TestJob.BACK_TO_BACK:
            env['CHALLENGER_OUT_URL'] = (
                'tcp://127.0.0.1:%s' % self.challenger_in_port)
            env['CHALLENGER_IN_URL'] = (
                'tcp://127.0.0.1:%s' % self.challenger_out_port)
            # Expose per-test B2B config if a b2b.yaml exists alongside test.py
            b2b_yaml = os.path.join(
                os.path.dirname(self.data.test_path), 'b2b.yaml')
            if os.path.exists(b2b_yaml):
                env['UXAS_B2B_CONFIG'] = b2b_yaml
        return {'output': os.path.join(RESULT_DIR, self.uid + '.out'),
                'ignore_environ': False,
                'env': env}


class TestData(object):
    """Handle test data related to a given test.

    This the data associated with each test job
    """

    def __init__(self, uid: str, test_path: str) -> None:
        """Initialize a test data.

        :param uid: the test uid
        :param test_path: path to the test.py file
        """
        self.uid = uid
        self.test_path = test_path

    def __str__(self) -> str:
        """Compute a string representation for display purpose."""
        return self.uid


class TestsuiteLoop(Walk):
    """The testsuite test scheduler."""

    def __init__(self, actions, jobs):
        self.jobs = jobs
        self.next_port = 5560
        super(TestsuiteLoop, self).__init__(actions)

    def create_job(self, uid, data, predecessors, notify_end):
        """See Walk.create_job doc."""
        return TestJob(uid, data, notify_end)

    def find_port(self) -> int:
        """Find the next available port.

        :return: an available port
        """
        port = None
        start_port = self.next_port

        # Do a maximum of 100 attempts.
        for _ in range(100):
            try:
                s = socket.socket()
                s.bind(('127.0.0.1', self.next_port))
                s.close()
                port = self.next_port
                self.next_port += 1
                break
            except OSError:
                self.next_port += 1

        if port is None:
            raise OSError("cannot find a valid port in range: %s - %s" %
                          (start_port, self.next_port - 1))
        logging.debug('allocate port: %s', port)
        return port

    def set_scheduling_params(self):
        """See Walk.set_scheduling_params doc."""
        super(TestsuiteLoop, self).set_scheduling_params()
        self.tokens = self.jobs

        # For back-to-back mode each job needs 4 ports (2 oracle + 2 challenger).
        # We allocate double to avoid immediate port reuse after release.
        ports_per_job = 4 if TestJob.BACK_TO_BACK else 2
        for _ in range(self.jobs * ports_per_job * 2):
            TestJob.PORTS.put(self.find_port())

        self.job_timeout = 60


def get_test_uid(path: str) -> str:
    """Compute the test uid from its path.

    :param path: path to the test.py implementing the test
    :return: an unique uid that does not contain path separators
    """
    return os.path.dirname(
        os.path.relpath(path,
                        TEST_DIR)).replace('/', '.').replace('\\', '.')


def get_test_list(service_filter=None) -> DAG:
    """Fetch the list of tests and return a DAG.

    :param service_filter: optional filter string. May be a service name
        prefix (e.g., 'arv') to include all tests under that service, or a
        full test UID (e.g., 'arv.service_status') to run exactly one test.
    :return: a dag representing the tests to perform.
    """
    test_dag = DAG()
    test_list = find(root=TEST_DIR, pattern='test.py')

    for test in test_list:
        test_uid = get_test_uid(test)
        # Include the test if:
        # - no filter specified, OR
        # - the filter is an exact UID match, OR
        # - the filter is a prefix of the UID (i.e., a service/group name)
        if (service_filter is None
                or test_uid == service_filter
                or test_uid.startswith(service_filter + '.')):
            test_dag.add_vertex(
                test_uid,
                data=TestData(uid=test_uid, test_path=test))
    return test_dag


def print_failure_summary() -> int:
    """Print the content of every failed test output file.

    A test is considered failed if its .out file either:
    - does not contain the word "OK" (test never reached the success print), or
    - contains "BackToBackMismatchError" (b2b comparison failed after assertions
      passed on the oracle side).

    :return: number of failures found
    """
    out_files = sorted(ls(os.path.join(RESULT_DIR, '*.out')))
    total = len(out_files)
    failures = []

    for path in out_files:
        with open(path) as fh:
            content = fh.read()
        if 'OK' not in content or 'BackToBackMismatchError' in content:
            uid = os.path.basename(path)[:-len('.out')]
            failures.append((uid, content))

    if failures:
        logging.info('')
        logging.info('=' * 60)
        logging.info('FAILURES (%d / %d)', len(failures), total)
        logging.info('=' * 60)
        for uid, content in failures:
            logging.info('')
            logging.info('--- FAIL: %s ---', uid)
            # Print content directly so indentation/newlines are preserved.
            print(content)
    else:
        logging.info('All %d tests passed.', total)

    return len(failures)


def main() -> int:
    """Main of the testsuite driver.

    :return: 0 in case of success
    """
    m = Main()
    m.argument_parser.add_argument(
        'service',
        nargs='?',
        default=None,
        help="optional filter: a service name (e.g., 'arv') to run all tests "
        "for that service, or a full test UID (e.g., 'arv.service_status') "
        "to run a single test")
    m.argument_parser.add_argument(
        '--impl',
        choices=['cpp', 'ada', 'both'],
        default='cpp',
        help="which UxAS implementation to test: 'cpp' (default), 'ada', or "
        "'both' for back-to-back comparison")
    m.argument_parser.add_argument(
        '--oracle',
        choices=['cpp', 'ada'],
        default='cpp',
        help="in back-to-back mode, which implementation is the oracle whose "
        "results are authoritative (default: 'cpp')")
    m.argument_parser.add_argument(
        '--challenger',
        choices=['cpp', 'ada'],
        default=None,
        help="in back-to-back mode, which implementation is the challenger "
        "being compared against the oracle (default: opposite of --oracle). "
        "Use --challenger=cpp with --oracle=cpp to run two C++ instances as a "
        "sanity check that the back-to-back infrastructure itself is correct.")
    m.argument_parser.add_argument(
        '--tolerance',
        type=float,
        default=1e-6,
        metavar='TOL',
        help="floating-point tolerance for back-to-back comparisons "
        "(default: 1e-6)")
    m.argument_parser.add_argument(
        '--ignore-fields',
        default='',
        metavar='FIELDS',
        help="comma-separated list of LMCP field names to ignore when "
        "comparing oracle and challenger outputs (e.g., 'SourceServiceID')")
    m.argument_parser.add_argument(
        '--source-dir',
        metavar="DIR",
        default=os.environ.get('UXAS_SOURCE_DIR'),
        help="root directory containing uxas sources. When set a coverage "
        "summary will be displayed. Default is the value of the env var "
        "UXAS_SOURCE_DIR: %s" % os.environ.get('UXAS_SOURCE_DIR'))
    m.argument_parser.add_argument(
        '--build-dir',
        metavar="DIR",
        default=os.environ.get('UXAS_BUILD_DIR'),
        help="root directory containing uxas build. When set a coverage "
        "summary will be displayed. Default is the value of the env var "
        "UXAS_BUILD_DIR: %s" % os.environ.get('UXAS_BUILD_DIR'))
    m.argument_parser.add_argument(
        '--jobs',
        type=int,
        default=Env().build.cpu.cores,
        help="Set parallelism (default: %s)" % Env().build.cpu.cores)
    m.argument_parser.add_argument(
        '--display-includes-coverage',
        default=False,
        action="store_true",
        help="If used coverage summary will show coverage information of "
        "include files (.h)")

    m.parse_args()

    try:
        import zmq  # noqa: F401 (ignore warning from flake8)
    except ImportError:
        logging.critical("zmp package is required. do pip install zmq")
        return 1

    # Validate that the required binaries are available.
    if m.args.impl in ('cpp', 'both'):
        uxas_bin = which('uxas')
        if not uxas_bin:
            logging.critical("uxas executable should be in the path")
            return 1
        logging.info("uxas found in %s", uxas_bin)
    if m.args.impl in ('ada', 'both'):
        install_dir = os.environ.get('UXAS_ADA_INSTALL_DIR', '')
        install_bin = os.path.join(install_dir, 'bin', 'uxas-ada') if install_dir else None
        uxas_ada_bin = (os.environ.get('UXAS_ADA_BIN') or
                        which('uxas-ada') or
                        (install_bin if install_bin and os.path.isfile(install_bin) else None))
        if not uxas_ada_bin:
            logging.critical(
                "uxas-ada executable not found; run with the uxas-ada anod "
                "environment active, add it to PATH, or set UXAS_ADA_BIN")
            return 1
        logging.info("uxas-ada found in %s", uxas_ada_bin)
        # Ensure the path is available to test subprocesses.
        os.environ['UXAS_ADA_BIN'] = uxas_ada_bin

    # Propagate implementation-selection settings to test subprocesses.
    os.environ['UXAS_IMPL'] = m.args.impl
    os.environ['UXAS_ORACLE'] = m.args.oracle
    challenger = m.args.challenger or ('ada' if m.args.oracle == 'cpp' else 'cpp')
    os.environ['UXAS_CHALLENGER'] = challenger
    os.environ['UXAS_B2B_TOLERANCE'] = str(m.args.tolerance)
    if m.args.ignore_fields:
        os.environ['UXAS_B2B_IGNORED_FIELDS'] = m.args.ignore_fields

    rm(RESULT_DIR, recursive=True)
    mkdir(RESULT_DIR)
    Env().add_search_path('PYTHONPATH', ROOT_DIR)

    # Ensure gcda are stored in the testsuite dir. It ensures that we don't
    # pollute uxas build dir and ease reset of coverage info on each run.
    if (m.args.source_dir and m.args.build_dir
            and len(find(m.args.build_dir, "*.gc*")) > 0):
        logging.info('Enable coverage mode')
        logging.info('Sources: %s', m.args.source_dir)
        logging.info('Objects: %s', m.args.build_dir)
        gcda_dir = os.path.join(RESULT_DIR, 'gcda')
        rm(gcda_dir, recursive=True)
        mkdir(gcda_dir)
        os.environ['GCOV_PREFIX'] = gcda_dir
        os.environ['GCOV_PREFIX_STRIP'] = \
            str(len(m.args.source_dir.split(os.sep)) - 1)

    TestJob.BACK_TO_BACK = (m.args.impl == 'both')
    TestsuiteLoop(actions=get_test_list(m.args.service), jobs=m.args.jobs)

    if (m.args.source_dir is not None and m.args.build_dir is not None
            and len(find(m.args.build_dir, "*.gc*")) > 0):
        # When a single service is specified, automatically show .h files
        # unless explicitly disabled.
        # For coverage, use only the top-level service name (first UID
        # component) so that 'arv.service_status' maps to 'arv', the same
        # as passing 'arv' directly.
        service_prefix = (m.args.service.split('.')[0]
                          if m.args.service else None)
        show_includes = m.args.display_includes_coverage or (m.args.service is not None)
        dump_gcov_summary(m.args.source_dir,
                          m.args.build_dir,
                          gcda_dir,
                          show_includes,
                          service_prefix)

    failures = print_failure_summary()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
