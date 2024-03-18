#!/bin/bash

# 
# This script installs CasADi with HSL linear solvers, METIS, 
# OOQP, WORHP, and a bit more. It ONLY works on ubuntu 22.04.
# 
# Do not share outside of FEB. Contains sensitive information.
# 
# Author: Reid Dye
# 

if [ `arch` == "aarch64" ]
then
	echo "M1 or M2 Mac detected! Switching installation script."
	echo "WORHP will NOT be installed on your system, and CasADi will be aquired from pip."
        echo "Triangle not installed - build fails on M1/M2 macs. Not sure why. Triangle source build coming soon but not yet implemented."
	curl -0fsSL "https://ocf.io/reiddye/casadi_installer_m1_m2_mac.sh";
	. casadi_installer_m1_m2_mac.sh;
	exit
fi

# Function to add a command to ~/.bashrc if it doesn't already exist
add_command_to_bashrc() {
  local command_to_add="$1"

  # Check if the command already exists in ~/.bashrc
  if grep -q "$command_to_add" "$HOME/.bashrc"; then
      echo "(i) Command '$command_to_add' already exists in ~/.bashrc"
  else
      # Add the command to ~/.bashrc
      echo -e "\n# Added by script\n$command_to_add" >> "$HOME/.bashrc"
      echo "(*) Command '$command_to_add' added to ~/.bashrc"
  fi
}

# install prereqs
sudo apt update -y
sudo apt upgrade -y
sudo apt --fix-broken install -y
sudo apt install -y curl git gcc g++ make cmake pkgconf libtool-bin ocl-icd-opencl-dev
sudo apt install -y libblas3 libblas-dev liblapack3 liblapack-dev gfortran
sudo apt install -y libblas-dev liblapack-dev libmetis-dev
sudo apt install -y texlive-latex-base texlive-binaries doxygen
sudo apt install -y check build-essential python3-dev python3-pip python3-numpy python3-scipy cython3 liblapack-dev libblas-dev python3-sphinx python3-sphinx-rtd-theme

# make references to `python` and `cython` work
sudo ln -s /usr/bin/python3 /usr/bin/python
sudo ln -s /usr/bin/cython3 /usr/bin/cython

# Triangle library
python -m pip install triangle

### HSL SOLVERS ###
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
cd ThirdParty-HSL
# get hsl solvers
tmpzip="$(mktemp)"
curl -o "$tmpzip" -f -L "https://www.ocf.berkeley.edu/~reiddye/coinhsl-2022.11.09.zip" || { echo "error: failed to download hsl zip archive"; rm -f "$tmpzip"; exit 1; }
unzip "$tmpzip"
mv coinhsl-2022.11.09 coinhsl


# configure and make and install HSL solvers
mkdir $HOME/hsl-install
./configure --prefix="$HOME/hsl-install/" # LIBS="-llapack" --with-blas="-L/usr/lib -lblas" CXXFLAGS="-g -O2 -fopenmp" FCFLAGS="-g -O2 -fopenmp" CFLAGS="-g -O2 -fopenmp"
make -j$(nproc)
sudo make install
# add link since casadi uses wrong name to find solvers
sudo ln -s "$HOME/hsl-install/lib/libcoinhsl.so" "$HOME/hsl-install/lib/coinhsl.so"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/hsl-install/lib"
add_command_to_bashrc 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/hsl-install/lib"'
export OMP_NUM_THREADS=$(nproc)
add_command_to_bashrc "export OMP_NUM_THREADS=$(nproc)"
source ~/.bashrc
cd ..

### OOQP ###
git clone https://github.com/emgertz/OOQP.git
cd OOQP
export MA27LIB=$HOME/hsl-install/lib/libcoinhsl.so
export MA57LIB=$HOME/hsl-install/lib/libcoinhsl.so
./configure --with-fortran CFLAGS="-fPIC" CXXFLAGS="-fPIC"
make -B
sudo make install
cd ..

### cowponder ###
curl -fsSL https://max.xz.ax/cowponder/cowponder_debian_installer.sh | sudo bash
cowponder -u >> /dev/null &
add_command_to_bashrc cowponder

### WORHP ###
# get deb file and install
tmpdeb="$(mktemp)"
curl -o "$tmpdeb" -f -L "https://www.ocf.berkeley.edu/~reiddye/worhp_1.15-0~ubuntu2204.deb" || { echo "error: failed to download WORHP deb"; rm -f "$tmpdeb"; exit 1; }
sudo dpkg -i "$tmpdeb"
sudo apt --fix-broken install --yes
rm -f "$tmpdeb"

# get license and link it properly
sudo curl -o "/usr/include/worhp/worhp.lic" -f -L "https://www.ocf.berkeley.edu/~reiddye/worhp.lic" || { echo "error: failed to download worhp license"; sudo rm -f "/usr/include/worhp/worhp.lic"; exit 1; }
sudo ln -s "/usr/include/worhp/worhp.lic" "/usr/worhp.lic"
sudo ln -s "/usr/include/worhp/worhp.lic" "/usr/lib/worhp.lic"

# funnies with the license. idk if this works
sudo sed -i "s/HOST NAME=\"reidslaptop\" MAC=\"08:9d:f4:26:79:f6\"/HOST NAME=\"$HOSTNAME\" MAC=\"08:9d:f4:26:79:f6\"/" "/usr/include/worhp/worhp.lic"
# it might or might not

add_command_to_bashrc "export WORHP_LICENSE_FILE=/usr/inclue/worhp/worhp.lic"
export WORHP_LICENSE_FILE="/usr/inclue/worhp/worhp.lic"

### TRLIB ###
git clone https://github.com/felixlen/trlib.git
cd trlib
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake -DTRLIB_BUILD_PYTHON3=ON .
make -j$(nproc)
make test
make doc
sudo make install
cd ../..

### HiGHS ###
git clone https://github.com/ERGO-Code/HiGHS.git
cd HiGHS
mkdir build
cd build
cmake ..
cmake --build .
sudo cmake --install .
cd ../..

### CasADi ###
# prereqs
sudo apt install -y gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
# prereqs for python interface
sudo apt install -y ipython3 python3-dev python3-numpy python3-scipy python3-matplotlib --install-recommends
sudo apt install -y swig --install-recommends

# get source
git clone https://github.com/casadi/casadi.git -b main casadi
rm -f casadi/casadi/interfaces/ooqp/ooqp_interface.cpp
curl -o "casadi/casadi/interfaces/ooqp/ooqp_interface.cpp" -f -L "https://www.ocf.berkeley.edu/~reiddye/ooqp_interface.cpp"
cd casadi; mkdir build; cd build; 

# make everything!!
add_command_to_bashrc 'export WORHP="/usr"'
add_command_to_bashrc "export HSL=\"$HOME/hsl-install/lib/\""
export WORHP="/usr"
export HSL="$HOME/hsl-install/lib/"

cmake \
	-DWITH_PYTHON=ON \
	-DWITH_PYTHON3=ON \
	-DWITH_IPOPT=ON \
	-DWITH_BUILD_IPOPT=ON \
	-DWITH_OPENMP=ON \
	-DWITH_THREAD=ON \
	-DWITH_OPENCL=ON \
	-DWITH_OOQP=ON \
	-DWITH_OSQP=ON \
	-DWITH_BUILD_OSQP=ON \
	-DWITH_BUILD_EIGEN3=ON \
	-DWITH_SUPERCS=ON \
	-DWITH_BUILD_SUPERCS=ON \
	-DWITH_PROXQP=ON \
	-DWITH_BUILD_PROXQP=ON \
	-DWITH_BUILD_DSDP=ON \
	-DWITH_DSDP=ON \
	-DWITH_BUILD_LAPACK=ON \
	-DWITH_LAPACK=ON \
	-DWITH_QPOASES=ON \
	-DWITH_NO_QPOASES_BANNER=ON \
	-DWITH_BLOCKSQP=ON \
	-DWITH_HIGHS=ON \
	-DWITH_SLEQP=OFF \
	-DWITH_BUILD_SLEQP=OFF \
	-DWITH_MUMPS=ON \
	-DWITH_HSL=ON \
	-DWITH_BLOCKSQP=ON \
	-DWITH_BUILD_REQUIRED=ON \
	-DWITH_WORHP=ON \
	-DWORHP_DIR="/usr/include/" \
	-DWITH_SPRAL=OFF \
	-DWITH_CLANG=OFF \
	-DHSL_DIR="$HOME/hsl-install/lib/" ..

make -j$(nproc)
sudo make install
cd ../..

echo "testing IPOPT..."
if python -c "import casadi as ca; x=ca.MX.sym('x'); exit(int(ca.nlpsol('testsolver', 'ipopt', {'f': x**2, 'x': x, 'g': x}, {'ipopt.print_level': 0, 'print_time': 0})(x0=10, lbg=1, ubg=20)['x'])==1);"; then
	echo "[[test passed]] IPOPT functional!"
else
	echo "[[test failed]] IPOPT borked!"
fi


echo "testing HSL MA57..."
if python -c "import casadi as ca; x=ca.MX.sym('x'); exit(int(ca.nlpsol('testsolver', 'ipopt', {'f': x**2, 'x': x, 'g': x}, {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.linear_solver': 'MA57'})(x0=10, lbg=1, ubg=20)['x'])==1);"; then
	echo "[[test passed]] HSL MA57 functional!"
else
	echo "[[test failed]] HSL MA57 borked!"
fi

echo "testing WORHP..."
if python -c "import casadi as ca; x=ca.MX.sym('x'); assert int(ca.nlpsol('testsolver', 'worhp', {'f': x**2, 'x': x, 'g': x}, {'print_time': 0, 'worhp': {'MA97blas3': True, 'MA97mf': True}})(x0=10, lbg=1, ubg=20)['x'])==1;"; then
	echo "[[test passed]] WORHP functional!"
else
	echo "[[test failed]] WORHP borked!"
fi
