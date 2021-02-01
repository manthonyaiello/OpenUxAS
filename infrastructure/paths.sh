# Globally-defined repository paths. This is a designed to work with BASH.
#
# This file should be sourced by any script that needs to work with repository
# paths: this way, if the layout changes, we can more easily manage the impact.

# The root of the OpenUxAS repository. For this, because we may get here via
# a source, we need to be careful about how we compute the path. Note no
# support for KSH at present.
if [[ -n $ZSH_EVAL_CONTEXT ]]; then
    OPENUXAS_ROOT="$( cd "${0:a:h}/.." && pwd )"
else
    OPENUXAS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. >/dev/null 2>&1 && pwd )"
fi

DOC_DIR="${OPENUXAS_ROOT}/doc"
EXAMPLES_DIR="${OPENUXAS_ROOT}/examples"
INFRASTRUCTURE_DIR="${OPENUXAS_ROOT}/infrastructure"
MDMS_DIR="${OPENUXAS_ROOT}/mdms"
OBJ_DIR="${OPENUXAS_ROOT}/obj"
RESOURCES_DIR="${OPENUXAS_ROOT}/resources"
SRC_DIR="${OPENUXAS_ROOT}/src"
TESTS_DIR="${OPENUXAS_ROOT}/tests"

CPP_DIR="${SRC_DIR}/cpp"
ADA_DIR="${SRC_DIR}/ada"

UXAS_BIN="${OBJ_DIR}/cpp/uxas"

# For LmcpGen and OpenAMASE
SUPPORT_DIR="${OPENUXAS_ROOT}/develop"
LMCP_DIR="${SUPPORT_DIR}/LmcpGen"
AMASE_DIR="${SUPPORT_DIR}/OpenAMASE"

VPYTHON_DIR="${OPENUXAS_ROOT}/.vpython"
VPYTHON_ACTIVATE="${VPYTHON_DIR}/bin/activate"

SBX_DIR="${INFRASTRUCTURE_DIR}/sbx"
