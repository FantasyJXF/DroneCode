############################################################################
#
# Copyright (c) 2015 - 2016 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# Enforce the presence of the GIT repository
#
# We depend on our submodules, so we have to prevent attempts to
# compile without it being present.
# 强制性保证GIT repository存在,由于于PX4依赖于其子模块，
# 所以必须防止在缺少子模块的情况下编译，从而有了此处的强制检查
# $(wildcard .git)用来获取工作路径下所有后缀为.git的文件，执行GIT检查
# ifeq...endif为条件判断

ifeq ($(wildcard .git),)
    $(error YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING.)
endif

CMAKE_VER := $(shell Tools/check_cmake.sh; echo $$?)
ifneq ($(CMAKE_VER),0)
    $(warning Not a valid CMake version or CMake not installed.)
    $(warning On Ubuntu 16.04, install or upgrade via:)
    $(warning )
    $(warning 3rd party PPA:)
    $(warning sudo add-apt-repository ppa:george-edison55/cmake-3.x -y)
    $(warning sudo apt-get update)
    $(warning sudo apt-get install cmake)
    $(warning )
    $(warning Official website:)
    $(warning wget https://cmake.org/files/v3.4/cmake-3.4.3-Linux-x86_64.sh)
    $(warning chmod +x cmake-3.4.3-Linux-x86_64.sh)
    $(warning sudo mkdir /opt/cmake-3.4.3)
    $(warning sudo ./cmake-3.4.3-Linux-x86_64.sh --prefix=/opt/cmake-3.4.3 --exclude-subdir)
    $(warning export PATH=/opt/cmake-3.4.3/bin:$$PATH)
    $(warning )
    $(error Fatal)
endif

# 这个Makefile主要是讲怎么传递参数，怎么编译px4固件，同时使用make和cmake是一样的
# Example usage:三种使用方法示例
# Help
# --------------------------------------------------------------------
# Don't be afraid of this makefile, it is just passing
# arguments to cmake to allow us to keep the wiki pages etc.
# that describe how to build the px4 firmware
# the same even when using cmake instead of make.
#
# Example usage:
#
# make px4fmu-v2_default 			(builds)
# make px4fmu-v2_default upload 	(builds and uploads)
# make px4fmu-v2_default test 		(builds and tests)
#
# This tells cmake to build the nuttx px4fmu-v2 default config in the
# directory build_nuttx_px4fmu-v2_default and then call make
# in that directory with the target upload.


# 以上的命令告诉cmake使用build/nuttx_px4fmu-v2_default的默认配置来build nuttx
# px4fmu，然后调用该路径下的make来上传target(编译好的文件)
#  明确设置默认的build目标，编译目标为all，编译规则为posix_sitl_default，如果make不指定编译
#  目标就会编译默认目标，此处输出的编译目标是px4fmu-v2_default
#  explicity set default build target
all: px4fmu-v2_default 

# Parsing
# --------------------------------------------------------------------
# assume 1st argument passed is the main target, the
# rest are arguments to pass to the makefile generated
# by cmake in the subdirectory
# 假设第传递的第一个参数是主目标，余下的参数是传输给子路径下的cmake产生的makefile的
# MAKECMDGOALS 就是输出的命令行参数px4fmu-v2_default
# FIRST_ARG 变量值就是px4fmu
FIRST_ARG := $(firstword $(MAKECMDGOALS))
# ARGS的变量值就是default
ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
#如果j为赋值，就将j赋值为4
j ?= 4

ifndef NO_NINJA_BUILD
NINJA_BUILD := $(shell ninja --version 2>/dev/null)
endif
ifdef NINJA_BUILD
    PX4_CMAKE_GENERATOR ?= "Ninja"
    PX4_MAKE = ninja
    PX4_MAKE_ARGS =
else

ifdef SYSTEMROOT
	# Windows
	PX4_CMAKE_GENERATOR ?= "MSYS Makefiles"
else
	PX4_CMAKE_GENERATOR ?= "Unix Makefiles"
endif
    #$(MAKE) 其实就是make命令
    PX4_MAKE = $(MAKE)
    #设置编译参数 -j4 --no-print-directory：同一时刻可同时允许执行4行命令，
    #禁止所有关于目录信息的打印
    PX4_MAKE_ARGS = -j$(j) --no-print-directory
endif

# check if replay env variable is set & set build dir accordingly
# 检查是否定义relay环境变量，并更改build目录的后缀：BUILD_DIR_SUFFIX
ifdef replay
	BUILD_DIR_SUFFIX := _replay
else
	BUILD_DIR_SUFFIX :=
endif

# additional config parameters passed to cmake
# 添加额外的配置参数
CMAKE_ARGS :=
ifdef EXTERNAL_MODULES_LOCATION
	CMAKE_ARGS := -DEXTERNAL_MODULES_LOCATION:STRING=$(EXTERNAL_MODULES_LOCATION)
endif

SRC_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
# 描述如何build一个cmake配置文件， 定义了cmke-build命令规则
define cmake-build
+@$(eval BUILD_DIR = $(SRC_DIR)/build_$@$(BUILD_DIR_SUFFIX))
+@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
+@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake .. -G$(PX4_CMAKE_GENERATOR) -DCONFIG=$(1) $(CMAKE_ARGS) || (cd .. && rm -rf $(BUILD_DIR)); fi
+@echo "PX4 CONFIG: $(BUILD_DIR)"
+@$(PX4_MAKE) -C "$(BUILD_DIR)" $(PX4_MAKE_ARGS) $(ARGS)
endef
#This命令是就是创建build文件夹，用cmake进行编译，然后再删除build文件夹
#本次编译就是走这个流程，注意这里的cmake编译的应该是CMakelist.txt文件，不是直接的.cmake文件
#$(2)应该是第二个参数，也就是传递的CMakelist.txt的路径，$(1)应该是传递的编译目标的名称，本次是nuttx_px4fmu-v2_default

define cmake-build-other
+@$(eval BUILD_DIR = $(SRC_DIR)/build_$@$(BUILD_DIR_SUFFIX))
+@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
+@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake $(2) -G$(PX4_CMAKE_GENERATOR) -DCONFIG=$(1) || (cd .. && rm -rf $(BUILD_DIR)); fi
+@$(PX4_MAKE) -C "$(BUILD_DIR)" $(PX4_MAKE_ARGS) $(ARGS)
endef

define colorecho
      @tput setaf 6
      @echo $1
      @tput sgr0
endef

# Get a list of all config targets.
# 获得所有cmake/configs/下所有的cmake目标文件，
# 本次编译的nuttx_px4fmu-v2_defualt就是从这里获得
ALL_CONFIG_TARGETS := $(basename $(shell find "$(SRC_DIR)/cmake/configs" ! -name '*_common*' ! -name '*_sdflight_*' -name '*.cmake' -print | sed  -e 's:^.*/::' | sort))
# Strip off leading nuttx_
# 去掉NUTTX_前缀并返回，这样NUTTX_CONFIG_TARGETS就是包含了所有关于NUTTX配置的一个组
NUTTX_CONFIG_TARGETS := $(patsubst nuttx_%,%,$(filter nuttx_%,$(ALL_CONFIG_TARGETS)))

# ADD CONFIGS HERE
# --------------------------------------------------------------------
#  Do not put any spaces between function arguments.

# All targets.
# All targets. 所有的目标
# $(ALL_CONFIG_TARGETS)指代了所有的编译目标，
# 每个目标都执行$(call cmake-build,$@,$(SRC_DIR))这个命令行
# 这是一种简便的书写模式
$(ALL_CONFIG_TARGETS):
	$(call cmake-build,$@)

# Abbreviated config targets.
# nuttx_ is left off by default; provide a rule to allow that. 只和nuttx相关的目标
# $(NUTTX_CONFIG_TARGETS):指代了所有的编译目标，在本次就是px4fmu-v2_default
# 每个目标都执行$(call cmake-build,$@,$(SRC_DIR))这个命令行,等价于
# $(call cmake-build,nuttx_px4fmu-v2_default,/f/px4/firmware),本次编译就是由此处进入到对
# nuttx_px4fmu-v2_default的编译

# nuttx_ is left off by default; provide a rule to allow that.
$(NUTTX_CONFIG_TARGETS):
	$(call cmake-build,nuttx_$@)

all_nuttx_targets: $(NUTTX_CONFIG_TARGETS)

#######################################################################################
#
#从此往后的就是一些伪目标，对本次编译来讲不重要，这些伪目标的作用有：
#1. 如clean执行清除任务，并不生成可执行目标，这种为目标的主要作用是执行命令
#2. 为了同时创建多个可执行目标，将一个为目标例如all作为终极目标，同时将
#   其依赖多伪目标
#
#######################################################################################

posix: posix_sitl_default
broadcast: posix_sitl_broadcast

# Multi- config targets.

eagle_default: posix_eagle_default qurt_eagle_default
eagle_legacy_default: posix_eagle_legacy_driver_default qurt_eagle_legacy_driver_default
excelsior_default: posix_excelsior_default qurt_excelsior_default

# Deprecated config targets.

ros_sitl_default:
	@echo "This target is deprecated. Use make 'posix_sitl_default gazebo' instead."

_sitl_deprecation:
	@echo "Deprecated. Use 'make posix_sitl_default jmavsim' or"
	@echo "'make posix_sitl_default gazebo' if Gazebo is preferred."

run_sitl_quad: _sitl_deprecation
run_sitl_plane: _sitl_deprecation
run_sitl_ros: _sitl_deprecation

# All targets with just dependencies but no recipe must either be marked as phony (or have the special @: as recipe).
.PHONY: all posix broadcast eagle_default eagle_legacy_default excelsior_default run_sitl_quad run_sitl_plane run_sitl_ros all_nuttx_targets

# Other targets
# --------------------------------------------------------------------

.PHONY: uavcan_firmware check check_format format unittest tests qgc_firmware package_firmware clean
.NOTPARALLEL:

# All targets with just dependencies but no recipe must either be marked as phony (or have the special @: as recipe).
.PHONY: checks_defaults checks_bootloaders checks_tests checks_alts checks_uavcan checks_sitls checks_last quick_check tests extra_firmware

uavcan_firmware:
ifeq ($(VECTORCONTROL),1)
	$(call colorecho,"Downloading and building Vector control (FOC) firmware for the S2740VC and PX4ESC 1.6")
	@(rm -rf vectorcontrol && git clone --quiet --depth 1 https://github.com/thiemar/vectorcontrol.git && cd vectorcontrol && BOARD=s2740vc_1_0 make --silent --no-print-directory && BOARD=px4esc_1_6 make --silent --no-print-directory && ../Tools/uavcan_copy.sh)
endif

check_px4fmu-v4_default: uavcan_firmware
check_px4fmu-v4_default_and_uavcan: check_px4fmu-v4_default
	@echo
ifeq ($(VECTORCONTROL),1)
	@echo "Cleaning up vectorcontrol firmware"
	@rm -rf vectorcontrol
	@rm -rf ROMFS/px4fmu_common/uavcan
endif

# All default targets that don't require a special build environment (currently built on semaphore-ci)
check: 	check_px4fmu-v1_default \
	check_px4fmu-v2_default \
	check_px4fmu-v2_test \
	check_px4fmu-v4_default_and_uavcan \
	check_mindpx-v2_default \
	check_posix_sitl_default \
	check_tap-v1_default \
	check_aerofc-v1_default \
	check_px4-stm32f4discovery_default \
	check_crazyflie_default \
	check_tests \
	check_format

# quick_check builds a single nuttx and posix target, runs testing, and checks the style
quick_check: check_posix_sitl_default check_px4fmu-v4_default check_tests check_format

check_format:
	$(call colorecho,"Checking formatting with astyle")
	@./Tools/check_code_style_all.sh
	@git diff --check

format:
	$(call colorecho,"Formatting with astyle")
	@./Tools/check_code_style_all.sh --fix

check_%:
	@echo
	$(call colorecho,"Building" $(subst check_,,$@))
	@$(MAKE) --no-print-directory $(subst check_,,$@)
	@echo

unittest: posix_sitl_default
	$(call cmake-build-other,unittest, ../unittests)
	@(cd build_unittest && ctest -j2 --output-on-failure)

run_tests_posix: posix_sitl_default
	@(cd build_posix_sitl_default/ && ctest -V)

tests: check_unittest run_tests_posix

# QGroundControl flashable firmware (currently built by travis-ci)
qgc_firmware: \
	check_px4fmu-v1_default \
	check_px4fmu-v2_default \
	check_px4fmu-v3_default \
	check_px4fmu-v4_default_and_uavcan \
	check_mindpx-v2_default \
	check_tap-v1_default \
	check_format

package_firmware:
	@zip --junk-paths Firmware.zip `find . -name \*.px4`

clean:
	@rm -rf build_*/
	-@$(MAKE) -C NuttX/nuttx clean

# All other targets are handled by PX4_MAKE. Add a rule here to avoid printing an error.
%:
	$(if $(filter $(FIRST_ARG),$@), \
		$(error "$@ cannot be the first argument. Use '$(MAKE) help|list_config_targets' to get a list of all possible [configuration] targets."),@#)

.PHONY: clean

CONFIGS:=$(shell ls cmake/configs | sed -e "s~.*/~~" | sed -e "s~\..*~~")

#help:
#	@echo
#	@echo "Type 'make ' and hit the tab key twice to see a list of the available"
#	@echo "build configurations."
#	@echo

empty :=
space := $(empty) $(empty)

# Print a list of non-config targets (based on http://stackoverflow.com/a/26339924/1487069)
help:
	@echo "Usage: $(MAKE) <target>"
	@echo "Where <target> is one of:"
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | \
		awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | \
		egrep -v -e '^[^[:alnum:]]' -e '^($(subst $(space),|,$(ALL_CONFIG_TARGETS) $(NUTTX_CONFIG_TARGETS)))$$' -e '_default$$' -e '^(posix|eagle|Makefile)'
	@echo
	@echo "Or, $(MAKE) <config_target> [<make_target(s)>]"
	@echo "Use '$(MAKE) list_config_targets' for a list of configuration targets."

# Print a list of all config targets.
list_config_targets:
	@for targ in $(patsubst nuttx_%,[nuttx_]%,$(ALL_CONFIG_TARGETS)); do echo $$targ; done

