# SW35xx_lib

A platform‑agnostic I²C library for controlling SW35xx‑series charger ICs (SW3518, SW3520, etc.) from any microcontroller or host.

## Features

- Read charging voltage and current  
- Detect and negotiate fast‑charge protocols (QC, PD)  
- Issue USB‑PD hard resets  
- Set custom current limits  
- Pluggable I²C bus interface for maximum portability  

## Requirements

- C++11 (or later) compiler 
- SW35xx‑series hardware wired to your platform’s I²C pins  

## License

This project is released under the **GNU GPL‑3.0**. See [LICENSE](LICENSE) for details.

## Credits

* **Original author:** [happyme531](https://github.com/happyme531)
* **Other contribution:** [twischer](https://github.com/twischer)
* **Generic I²C bus integration:** [DaddyNequis](https://github.com/DaddyNequis/h1_SW35xx/tree/feature/GenericI2CBus) (commit 9e918964328f9e13dcf0eba52721fd4597b52b37)
