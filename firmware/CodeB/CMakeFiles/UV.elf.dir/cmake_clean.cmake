file(REMOVE_RECURSE
  "config/sdkconfig.h"
  "config/sdkconfig.cmake"
  "bootloader/bootloader.elf"
  "bootloader/bootloader.bin"
  "bootloader/bootloader.map"
  "UV.bin"
  "UV.map"
  "project_elf_src.c"
  "project_elf_src.c"
  "CMakeFiles/UV.elf.dir/project_elf_src.c.obj"
  "UV.elf.pdb"
  "UV.elf"
  "UV.elf.manifest"
)

# Per-language clean rules from dependency scanning.
foreach(lang C)
  include(CMakeFiles/UV.elf.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
