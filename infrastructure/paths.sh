# Globally-defined repository paths. This is a designed to work with BASH.
#
# This file should be sourced by any script that needs to work with repository
# paths: this way, if the layout changes, we can more easily manage the impact.

# The root of the OpenUxAS repository. For this, because we may get here via
# a source, we need to be careful about how we compute the path. Note no
# support for KSH at present.
if [[ -n $ZSH_EVAL_CONTEXT ]]; then
    export OPENUXAS_ROOT="$( cd "${0:a:h}/.." && pwd )"
else
    export OPENUXAS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. >/dev/null 2>&1 && pwd )"
fi

export ANOD_BIN="${OPENUXAS_ROOT}/anod"

export DOC_DIR="${OPENUXAS_ROOT}/doc"
export EXAMPLES_DIR="${OPENUXAS_ROOT}/examples"
export INFRASTRUCTURE_DIR="${OPENUXAS_ROOT}/infrastructure"
export MDMS_DIR="${OPENUXAS_ROOT}/mdms"
export OBJ_DIR="${OPENUXAS_ROOT}/obj"
export RESOURCES_DIR="${OPENUXAS_ROOT}/resources"
export SRC_DIR="${OPENUXAS_ROOT}/src"
export TESTS_DIR="${OPENUXAS_ROOT}/tests"

export CPP_DIR="${SRC_DIR}/cpp"
export ADA_DIR="${SRC_DIR}/ada"

export UXAS_BIN="${OBJ_DIR}/cpp/uxas"

# For LmcpGen and OpenAMASE
export SUPPORT_DIR="${OPENUXAS_ROOT}/develop"
export LMCP_DIR="${SUPPORT_DIR}/LmcpGen"
export AMASE_DIR="${SUPPORT_DIR}/OpenAMASE"

export VPYTHON_DIR="${OPENUXAS_ROOT}/.vpython"
export VPYTHON_ACTIVATE="${VPYTHON_DIR}/bin/activate"

export INSTALL_LIBEXEC_DIR="${INFRASTRUCTURE_DIR}/install-libexec"

export SPEC_DIR="${INFRASTRUCTURE_DIR}/specs"
export SBX_DIR="${INFRASTRUCTURE_DIR}/sbx"
