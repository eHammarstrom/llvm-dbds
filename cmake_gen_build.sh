TARGET=X86
INSTALL_DIR=/opt/llvm/release_70

cd ..
mkdir build
cd build

cmake ../src -DCMAKE_BUILD_TYPE=Release \
  -DLLVM_ENABLE_ASSERTIONS=ON \
  -DLLVM_TARGETS_TO_BUILD=$TARGET \
  -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DLLVM_USE_SANITIZER=Address


  #-DLLVM_BINUTILS_INCDIR=../../binutils/include # builds LLVMgold.so
  #-DLLVM_BUILD_DOCS=ON \
  #-DLLVM_ENABLE_DOXYGEN=ON \
