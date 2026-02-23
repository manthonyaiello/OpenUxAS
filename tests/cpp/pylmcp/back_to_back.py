"""Back-to-back testing support for comparing C++ and Ada UxAS implementations.

BackToBackServer runs both implementations simultaneously with the same input
and compares their outputs. One implementation is designated the oracle (its
results are authoritative and returned to the test's assertion logic); the
other is the challenger (its results are compared against the oracle's).

Controlled by environment variables set by run-tests.py:
  UXAS_ORACLE              'cpp' or 'ada' (default: 'cpp')
  UXAS_CHALLENGER          'ada' or 'cpp' (default: 'ada')
  UXAS_B2B_TOLERANCE       float tolerance for floating-point comparison
                           (default: '1e-6')
  UXAS_B2B_IGNORED_FIELDS  comma-separated field names to skip in comparison
                           (default: '')
  UXAS_B2B_CONFIG          path to a per-test b2b.yaml configuration file
                           (set automatically by run-tests.py when present)
  CHALLENGER_OUT_URL       URL for challenger hub-input PULL socket
  CHALLENGER_IN_URL        URL for challenger hub-output PUB socket

Per-test configuration (b2b.yaml)
----------------------------------
A test directory may contain a b2b.yaml file to customise B2B comparison for
that test alone.  Supported keys:

  ignore_fields: [FieldName, ...]
      Additional field names to skip at any nesting level (supplements the
      global --ignore-fields flag).

  field_rules:
      "path.with[*].wildcards":
          normalize:
              - pattern: '<regex>'
                replacement: '<string>'
          ...
      Path keys use the same dotted notation produced by compare_objects.
      [*] matches any list index.  When a normalize rule matches, both the
      oracle and challenger values are converted to strings, each substitution
      is applied in order with re.sub, and the resulting strings are compared.
      Values that are equal after normalization are not reported as a mismatch.

  xfail: true
      Mark this test as an expected failure in B2B mode.  A test that fails
      with a BackToBackMismatchError is reported as XFAIL rather than FAIL,
      and the overall test run still exits with code 0.  If the test passes
      unexpectedly (no mismatch), it is reported as XPASS and counted as an
      error.

  xfail_match: '<substring>'
      Narrows the xfail scope: XFAIL only applies when the
      BackToBackMismatchError message contains this substring.  A failure
      whose message does not match is reported as a regular FAIL so new bugs
      are not silently hidden.  Omit to accept any BackToBackMismatchError.
"""
import os
import re
import typing

from pylmcp.message import Message
from pylmcp import Object
from pylmcp.uxas import UxASConfig
from pylmcp.server import Server, AdaServer, ServerTimeout, DEFAULT_IN_URL, DEFAULT_OUT_URL


class BackToBackMismatchError(Exception):
    """Raised when oracle and challenger produce different outputs."""
    pass


def _path_key_to_regex(key: str) -> 're.Pattern[str]':
    """Compile a field_rules path key to a regex pattern.

    [*] in the key matches any list index (e.g. Info[*].Value matches
    Info[0].Value, Info[1].Value, etc.).
    All other regex metacharacters are treated as literals.
    """
    escaped = re.escape(key)
    # re.escape turns [*] into \[\*\]; replace with \[\d+\] to match indices.
    pattern = escaped.replace(r'\[\*\]', r'\[\d+\]')
    return re.compile(r'^' + pattern + r'$')


def _compile_field_rules(field_rules: dict) -> list:
    """Pre-compile field_rules path keys to regex patterns.

    Returns a list of (compiled_path_pattern, rule_dict) pairs.
    """
    return [(_path_key_to_regex(key), rule)
            for key, rule in field_rules.items()]


def _make_server(impl: str, out_url: str, in_url: str,
                 bridge_cfg: typing.Union[None, UxASConfig],
                 entity_id: int):
    """Construct the appropriate server type for the given impl.

    Bypasses Server.__new__ dispatch to directly instantiate the right class.
    """
    if impl == 'ada':
        return AdaServer(out_url=out_url, in_url=in_url,
                         bridge_cfg=bridge_cfg, entity_id=entity_id)
    else:
        # Temporarily clear UXAS_IMPL so Server.__new__ does not recurse.
        saved = os.environ.get('UXAS_IMPL', 'cpp')
        os.environ['UXAS_IMPL'] = 'cpp'
        try:
            return Server(out_url=out_url, in_url=in_url,
                          bridge_cfg=bridge_cfg, entity_id=entity_id)
        finally:
            os.environ['UXAS_IMPL'] = saved


class BackToBackServer(object):
    """Run oracle and challenger UxAS implementations simultaneously.

    Provides the same interface as Server. send_msg forwards messages to both
    implementations. wait_for_msg waits for both, compares outputs, and
    returns the oracle's message (so existing test assertions remain valid).

    Mismatches accumulate and are raised as BackToBackMismatchError when the
    context manager exits (via stop()).

    Round-trip request/response handling
    -------------------------------------
    When oracle and challenger use different RequestID counters (e.g. Ada ARV
    intentionally starts at 10_000 to avoid conflicts in production), a
    UniqueAutomationResponse sent with the oracle's RequestID would be ignored
    by the challenger (wrong ResponseID).

    BackToBackServer handles this transparently:
    - wait_for_msg('UniqueAutomationRequest') records oracle_id→challenger_id
    - send_msg(UniqueAutomationResponse) sends the oracle its original message
      and sends the challenger a copy with its own mapped ResponseID.
    """

    def __init__(self,
                 out_url: str = DEFAULT_IN_URL,
                 in_url: str = DEFAULT_OUT_URL,
                 challenger_out_url: str = DEFAULT_IN_URL,
                 challenger_in_url: str = DEFAULT_OUT_URL,
                 bridge_cfg: typing.Union[None, UxASConfig] = None,
                 entity_id: int = 100):
        """Initialise oracle and challenger servers.

        :param out_url: oracle hub-input URL (IN_SERVER_URL for oracle)
        :param in_url: oracle hub-output URL (OUT_SERVER_URL for oracle)
        :param challenger_out_url: challenger hub-input URL
        :param challenger_in_url: challenger hub-output URL
        :param bridge_cfg: service configuration (same for both)
        :param entity_id: entity id (same for both)
        """
        oracle_impl = os.environ.get('UXAS_ORACLE', 'cpp')
        challenger_impl = os.environ.get('UXAS_CHALLENGER', 'ada')
        self.tolerance = float(os.environ.get('UXAS_B2B_TOLERANCE', '1e-6'))
        ignored_raw = os.environ.get('UXAS_B2B_IGNORED_FIELDS', '')
        self.ignored_fields = set(
            f.strip() for f in ignored_raw.split(',') if f.strip())
        self.field_rules = []  # type: typing.List[typing.Tuple]
        self.mismatches = []  # type: typing.List[str]

        # Load per-test configuration from b2b.yaml if present.
        b2b_config_path = os.environ.get('UXAS_B2B_CONFIG', '')
        if b2b_config_path:
            import yaml
            with open(b2b_config_path) as fh:
                cfg = yaml.safe_load(fh) or {}
            for field in cfg.get('ignore_fields', []):
                self.ignored_fields.add(field.strip())
            self.field_rules = _compile_field_rules(
                cfg.get('field_rules', {}))
        # Maps oracle UniqueAutomationRequest.RequestID to challenger's.
        # Used by send_msg to substitute the correct ResponseID so the
        # challenger processes UniqueAutomationResponse messages correctly.
        self._request_id_map = {}  # type: typing.Dict[int, int]

        self.oracle = _make_server(oracle_impl, out_url, in_url,
                                   bridge_cfg, entity_id)
        self.challenger = _make_server(challenger_impl,
                                       challenger_out_url, challenger_in_url,
                                       bridge_cfg, entity_id)

    def send_msg(self, msg):
        """Send a message to both oracle and challenger.

        For UniqueAutomationResponse messages whose ResponseID was captured
        from a previous wait_for_msg('UniqueAutomationRequest') call, the
        oracle receives the original message while the challenger receives a
        copy with its own mapped ResponseID.  This handles implementations
        that start their RequestID counter at different values (e.g. Ada ARV
        starts at 10_000 intentionally).

        :param msg: a UxAS message or an LMCP Object
        :type msg: pylmcp.message.Message | pylmcp.Object
        """
        # Detect UniqueAutomationResponse with a mapped ResponseID.
        obj = msg if isinstance(msg, Object) else getattr(msg, 'obj', None)
        if (self._request_id_map and obj is not None
                and hasattr(obj, 'object_class')
                and obj.object_class is not None
                and 'UniqueAutomationResponse' in obj.object_class.full_name):
            response_id = obj.data.get('ResponseID')
            if response_id in self._request_id_map:
                chal_id = self._request_id_map[response_id]
                self.oracle.send_msg(msg)
                # Build a shallow copy with the challenger's ResponseID.
                chal_obj = Object(class_name=obj.object_class)
                chal_obj.data = dict(obj.data)
                chal_obj.data['ResponseID'] = chal_id
                self.challenger.send_msg(chal_obj)
                return
        self.oracle.send_msg(msg)
        self.challenger.send_msg(msg)

    def wait_for_msg(self, descriptor=None, timeout=10.0):
        """Wait for matching messages from both implementations and compare.

        Returns the oracle's message so existing test assertions remain valid.
        Challenger mismatches are recorded and raised at stop()/context exit.

        When descriptor is a UniqueAutomationRequest, the oracle's and
        challenger's RequestID values are recorded in _request_id_map so that
        a subsequent send_msg(UniqueAutomationResponse) can substitute the
        correct ResponseID for the challenger.

        :param descriptor: if not None, wait for a message matching this type
        :param timeout: timeout in seconds for each implementation
        :return: the oracle's message
        :rtype: pylmcp.message.Message
        """
        oracle_msg = self.oracle.wait_for_msg(descriptor=descriptor,
                                              timeout=timeout)
        try:
            challenger_msg = self.challenger.wait_for_msg(descriptor=descriptor,
                                                          timeout=timeout)
            self._compare(oracle_msg, challenger_msg, descriptor)
            # Track RequestID mapping so round-trip responses use the right ID.
            if (descriptor is not None
                    and 'UniqueAutomationRequest' in descriptor):
                oracle_id = oracle_msg.obj.data.get('RequestID')
                chal_id = challenger_msg.obj.data.get('RequestID')
                if oracle_id is not None and chal_id is not None:
                    self._request_id_map[oracle_id] = chal_id
        except ServerTimeout:
            self.mismatches.append(
                'Challenger timed out waiting for %s' % descriptor)

        return oracle_msg

    def _compare(self, oracle_msg, challenger_msg, descriptor):
        """Compare oracle and challenger messages, recording any differences."""
        diffs = compare_objects(oracle_msg.obj, challenger_msg.obj,
                                tolerance=self.tolerance,
                                ignored_fields=self.ignored_fields,
                                field_rules=self.field_rules)
        if diffs:
            label = descriptor or oracle_msg.descriptor
            self.mismatches.append(
                'Mismatch for %s:\n%s' % (label, '\n'.join(diffs)))

    def stop(self):
        """Stop both servers and raise if any mismatches were found."""
        self.oracle.stop()
        self.challenger.stop()
        if self.mismatches:
            raise BackToBackMismatchError(
                'Back-to-back comparison failed:\n\n' +
                '\n\n'.join(self.mismatches))

    def __del__(self):
        # Best-effort cleanup; don't raise from __del__.
        try:
            self.oracle.stop()
            self.challenger.stop()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _tb):
        self.stop()


def compare_objects(a, b, tolerance: float, ignored_fields: set,
                    path: str = '',
                    field_rules: list = None) -> typing.List[str]:
    """Recursively compare two LMCP Objects or values.

    Returns a list of human-readable difference strings. An empty list means
    the two values are considered equal (within tolerance for floats).

    :param a: oracle value (Object, dict, list, or scalar)
    :param b: challenger value
    :param tolerance: absolute+relative tolerance for float comparisons
    :param ignored_fields: set of field names to skip at any nesting level
    :param path: dotted path for error messages (built during recursion)
    :param field_rules: pre-compiled list of (path_regex, rule_dict) pairs
        from the per-test b2b.yaml; applied at scalar comparison points
    """
    if field_rules is None:
        field_rules = []
    diffs = []

    # Unwrap pylmcp Object instances to their data dicts.
    a_data = a.data if hasattr(a, 'data') and hasattr(a, 'object_class') else a
    b_data = b.data if hasattr(b, 'data') and hasattr(b, 'object_class') else b

    # If both sides are Objects, also check that their class names agree.
    if (hasattr(a, 'object_class') and hasattr(b, 'object_class') and
            a.object_class is not b.object_class):
        diffs.append('%s: class mismatch: %s vs %s' % (
            path or '<root>',
            a.object_class.full_name if a.object_class else None,
            b.object_class.full_name if b.object_class else None))
        return diffs  # No point comparing fields of different classes.

    if isinstance(a_data, dict) and isinstance(b_data, dict):
        all_keys = set(a_data) | set(b_data)
        for key in sorted(all_keys):
            field_path = '%s.%s' % (path, key) if path else key
            if key in ignored_fields:
                continue
            if key not in a_data:
                diffs.append('%s: missing from oracle' % field_path)
            elif key not in b_data:
                diffs.append('%s: missing from challenger' % field_path)
            else:
                diffs.extend(compare_objects(a_data[key], b_data[key],
                                             tolerance, ignored_fields,
                                             field_path, field_rules))
        return diffs

    if isinstance(a_data, list) and isinstance(b_data, list):
        if len(a_data) != len(b_data):
            diffs.append('%s: list length %d vs %d' % (
                path or '<root>', len(a_data), len(b_data)))
            # Still compare common prefix so we get more detail.
        for i, (av, bv) in enumerate(zip(a_data, b_data)):
            diffs.extend(compare_objects(av, bv, tolerance, ignored_fields,
                                         '%s[%d]' % (path, i), field_rules))
        return diffs

    # Scalar comparison.
    a_val = a_data
    b_val = b_data

    # Check per-test field rules for this path.
    for pattern, rule in field_rules:
        if pattern.match(path):
            normalize_steps = rule.get('normalize', [])
            if normalize_steps:
                a_norm = str(a_val)
                b_norm = str(b_val)
                for step in normalize_steps:
                    a_norm = re.sub(step['pattern'],
                                    step.get('replacement', ''), a_norm)
                    b_norm = re.sub(step['pattern'],
                                    step.get('replacement', ''), b_norm)
                if a_norm != b_norm:
                    diffs.append('%s: %r vs %r (after normalization)' % (
                        path or '<root>', a_norm, b_norm))
            return diffs  # Rule handled this path; skip default comparison.

    if isinstance(a_val, float) or isinstance(b_val, float):
        try:
            fa, fb = float(a_val), float(b_val)
            scale = max(1.0, abs(fa), abs(fb))
            if abs(fa - fb) <= tolerance * scale:
                return diffs  # within tolerance
        except (TypeError, ValueError):
            pass
        diffs.append('%s: %.10g vs %.10g (tolerance %.2g)' % (
            path or '<root>', a_val, b_val, tolerance))
    elif a_val != b_val:
        diffs.append('%s: %r vs %r' % (path or '<root>', a_val, b_val))

    return diffs
