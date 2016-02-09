#!/usr/bin/env bash

set -e -u
set -o pipefail

# on linux we need to upgrade to a c++11 compatible compiler
if [[ $(uname -s) == 'Linux' ]]; then
  # when building on sudo:false/docker machines we can speedup
  # builds by using ccache. The tricky below is to avoid cmake
  # breaking when trying to validate the compiler works
  if [[ ${TRAVIS_BUILD_TYPE} == 'MASON' ]]; then
    echo '#!/bin/bash' > ./cxx_compiler;
    echo 'ccache ${CXX} "$@"' >> ./cxx_compiler;
    chmod +x ./cxx_compiler;
    export CXX="$(pwd)/cxx_compiler";
    echo '#!/bin/bash' > ./c_compiler;
    echo 'ccache ${CC} "$@"' >> ./c_compiler;
    chmod +x ./c_compiler;
    export CC="$(pwd)/c_compiler";
  fi
fi

set +e +u