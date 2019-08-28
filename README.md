# OperatorInterface
Code for 6328's custom operator interface boards

## Cloning
Clone with `git clone --recurse-submodules` or run `git submodule init` and `git submodule update` after cloning.

## VS Code
To build in VS Code with the Arduino extension, put the following in .vscode/c_cpp_properties.json, replacing `\\` with `/` if not on Windows:
```json
{
    "configurations": [
        {
            "name": "<Win32, Mac, Linux>",
            "includePath": [
                "<Arduino Install Path>\\tools\\**",
                "<Arduino Install Path>\\hardware\\arduino\\avr\\libraries\\**",
                "<Arduino Install Path>\\hardware\\arduino\\avr\\cores\\**",
                "<Arduino Install Path>\\hardware\\arduino\\avr\\variants\\**",
                "<Arduino Install Path>\\hardware\\tools\\avr\\avr\\include\\**"
            ],
            "forcedInclude": [
                "<Arduino Install Path>\\hardware\\arduino\\avr\\cores\\arduino\\Arduino.h"
            ],
            "intelliSenseMode": "gcc-x64",
            "compilerPath": "<Your Compiler Path>",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "defines": ["ARDUINO_ARCH_AVR", "ARDUINO=10809", "USBCON"]
        }
    ],
    "version": 4
}
```
