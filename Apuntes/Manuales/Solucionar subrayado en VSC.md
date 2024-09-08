1. En VSC, abrir el archivo *c_cpp_properties.json* de *.vscode*
   > También se puede buscar con Ctrl + Shift + P y buscar *C/C++: Editar configuraciones (JSON)*
2. Editar el archivo por:
   ```
   {
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "${workspaceFolder}/include"      
            ],
            "defines": [
                "STM32F1"                            
            ],
            "compilerPath": "/opt/gcc-arm/bin/arm-none-eabi-gcc",
            "compilerArgs": [
                "-mcpu=cortex-m3",                    
                "-mthumb",                            
                "-msoft-float",                       
                "-mfix-cortex-m3-ldrd",               
                "-DSTM32F1"                           
            ],
            "cStandard": "c99",                       
            "cppStandard": "gnu++17",
            "intelliSenseMode": "linux-gcc-arm"
        }
    ],
    "version": 4
   }
  ```
