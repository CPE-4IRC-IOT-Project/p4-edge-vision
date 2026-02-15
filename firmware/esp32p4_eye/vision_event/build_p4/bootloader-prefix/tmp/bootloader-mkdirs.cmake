# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/valentine/esp/esp-idf/components/bootloader/subproject"
  "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader"
  "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader-prefix"
  "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader-prefix/tmp"
  "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader-prefix/src/bootloader-stamp"
  "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader-prefix/src"
  "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/valentine/CPE_LYON/S8/Projet_IOT/esp-dev-kits/examples/esp32-p4-eye/examples/factory_demo/build_p4/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
