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
  CHALLENGER_OUT_URL       URL for challenger hub-input PULL socket
  CHALLENGER_IN_URL        URL for challenger hub-output PUB socket
"""
import os
import typing

from pylmcp.message import Message
from pylmcp import Object
from pylmcp.uxas import UxASConfig
from pylmcp.server import Server, AdaServer, ServerTimeout, DEFAULT_IN_URL, DEFAULT_OUT_URL


class BackToBackMismatchError(Exception):
    """Raised when oracle and challenger produce different outputs."""
    pass


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
        self.mismatches = []  # type: typing.List[str]
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
                                ignored_fields=self.ignored_fields)
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
                    path: str = '') -> typing.List[str]:
    """Recursively compare two LMCP Objects or values.

    Returns a list of human-readable difference strings. An empty list means
    the two values are considered equal (within tolerance for floats).

    :param a: oracle value (Object, dict, list, or scalar)
    :param b: challenger value
    :param tolerance: absolute+relative tolerance for float comparisons
    :param ignored_fields: set of field names to skip at any nesting level
    :param path: dotted path for error messages (built during recursion)
    """
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
                                             field_path))
        return diffs

    if isinstance(a_data, list) and isinstance(b_data, list):
        if len(a_data) != len(b_data):
            diffs.append('%s: list length %d vs %d' % (
                path or '<root>', len(a_data), len(b_data)))
            # Still compare common prefix so we get more detail.
        for i, (av, bv) in enumerate(zip(a_data, b_data)):
            diffs.extend(compare_objects(av, bv, tolerance, ignored_fields,
                                         '%s[%d]' % (path, i)))
        return diffs

    # Scalar comparison.
    a_val = a_data
    b_val = b_data

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
