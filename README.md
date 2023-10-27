<p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177659981-d0c4ffe2-3738-45ec-886e-c289925b0546.png" height="200" align="center">
</p>

![Status](https://img.shields.io/static/v1?style=flat&logo=github&label=status&message=active&color=blue) [![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](http://makeapullrequest.com)  


This repository stores an **open optical tracking/motion capture system using passive reflective markers and Raspberry Pi**. The system has a 1cm accuracy when using 4 cameras, but the number can be expanded to improve cover. 

## 🔖 Docs

- Master thesis: ["Heterogeneous optical tracking system for mobile vehicles"](https://drive.google.com/file/d/1Gvj34tuTL8okl7SSPtr6r7JbKLNrIH3P/view?usp=sharing)
- Paper: ["Optical tracking system based on COTS components"](https://ieeexplore.ieee.org/document/10053039)

## 🏗️ Requirements

- Two (*minimum*) Raspberry 3b or above
- Two (*minimum*) NoIR cameras 
    * **this code uses V2 `picamera` module** (> 90FPS)
    * For V3 modules, use [@adthoms fork](https://github.com/adthoms/MoCapRasp/tree/main)
- Ethernet cables
- Tripods and [gymbals](https://www.amazon.com.br/gp/product/B099HPMZK1/ref=ppx_yo_dt_b_asin_title_o02_s01?ie=UTF8&psc=1))
- Dissasembled PC coolers (there will be frequency capping if no ventilation)
- IR LED [rings](https://produto.mercadolivre.com.br/MLB-2096109150-led-infravermelho-cameras-seguranca-com-sensor-kit-4-placa-_JM) or [reflectors](https://produto.mercadolivre.com.br/MLB-705743885-refletor-72-leds-infravermelho-para-camera-de-seguranca-_JM#position=18&search_layout=stack&type=item&tracking_id=f82f63b5-6055-4f00-a978-0f2bfc703d91) (rings are easier to align with the camera axis)
- IR low-pass filters (floppy disks or [glass](https://pt.aliexpress.com/item/1005003709944263.html?spm=a2g0o.order_list.0.0.1856caa4oP6TAy&gatewayAdapt=glo2bra))
- NPN FET, optocoupler, and 3x 1kΩ resistors for LED turn on/off circuit 
- Polystyrene spheres 
- [Reflective adehesive](https://dmrefletivos.com.br/sinalizacao-viaria/grau-comercial/)
  
## ⚔️ Usage

Please follow each folder `README.md` guide file. 
    
- [`./calib`](/calib/) for intrinsics calibration 
- [`./server`](/server/) for the server
- [`./client`](/client/) for the client]
