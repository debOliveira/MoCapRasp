<p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177659981-d0c4ffe2-3738-45ec-886e-c289925b0546.png" height="200" align="center">
</p>

![Status](https://img.shields.io/static/v1?style=flat&logo=github&label=status&message=active&color=blue) [![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](http://makeapullrequest.com)  
   

This repository stores an **open optical tracking/motion capture system using passive reflective markers and Raspberry Pi**. 

> - This work is an result of my master dissertation titled ["Heterogeneous optical tracking system for mobile vehicles"](https://drive.google.com/file/d/1Gvj34tuTL8okl7SSPtr6r7JbKLNrIH3P/view?usp=sharing). 
> - An article titled ["Optical tracking system based on COTS components"](https://ieeexplore.ieee.org/document/10053039) is also at the IPAS 2022. 

## Usage

Each folder has its own `README.md` guide file. Please read each one and adjust both your server and clients. 

## Organization
    
- [`./calib`](/calib/) stores the code to collect the images used in intrinsics calibration. 
- [`./server`](/server/) has the codes of the server side.
- [`./client`](/client/) has the codes of the client side and comprises the files used in the clients. 

## Materials

- Two (*minimum*) Raspberry 3b or above
- Two (*minimum*) NoIR cameras 
    * **V2 cameras are preferable** if it's required more than 90FPS
- Long Ethernet cables that cover from the router/switcher to each RPi
- High support, like tripods or shelfs (do not forget to get some [gymbals](https://www.amazon.com.br/gp/product/B099HPMZK1/ref=ppx_yo_dt_b_asin_title_o02_s01?ie=UTF8&psc=1))
- Dissasembled PC collers, one for each RPi
- IR LED [rings](https://produto.mercadolivre.com.br/MLB-2096109150-led-infravermelho-cameras-seguranca-com-sensor-kit-4-placa-_JM) or [reflectors](https://produto.mercadolivre.com.br/MLB-705743885-refletor-72-leds-infravermelho-para-camera-de-seguranca-_JM#position=18&search_layout=stack&type=item&tracking_id=f82f63b5-6055-4f00-a978-0f2bfc703d91) :arrow_right: *Rings are easier to align with the camera axis*
- IR low-pass filters :arrow_right: *using floppy disks or [glass](https://pt.aliexpress.com/item/1005003709944263.html?spm=a2g0o.order_list.0.0.1856caa4oP6TAy&gatewayAdapt=glo2bra)*
- Polystyrene spheres 
- [Reflective adehesive](https://dmrefletivos.com.br/sinalizacao-viaria/grau-comercial/)
- Hard cardboard to construct the camera boxes.
- NPN FET, optocoupler, 3x 1kΩ resistors for each turn on/off circuit 
    * You can also weld the power direct to the led ring and manually turn it on or off
    
## Construction

### Camera

1) Connect the camera to a RPi and capture a calibration data set using [`./calib`](/calib/). 
2)  Run [camera calibrator application](https://github.com/debOliveira/myCameraCalibrator).
3) Construct the boxes to the cameras using cardboard. 
    * I used a 8cm X 8cm X 5cm box. 
    * Remeber to cut a hole in the center for the camera lens and the LED power input, and at the back for the flat cable and power cable of the LED ring.
    * I cutted a 3mm X 3mm square at the bottom to fit the gymbal tip. 
    * Then I did four little holes to pass a string to hold the box to the gymbal base.
    * Leave the front open.
5) Weld the LED to the power source or connect it to the turn on/off circuit pins. 
6) Put the filter over the camera lens and cover the rest with insulating tape.
7) Put the camera+filter back at the box, check if the LED is powering on and fit the gymbal tip through the hole at the bottom of the box. 
8) Pass the string behind the gymbal bar and tie a not to secure it over the gymbal base.
10) Double check to avoid opening the box multiple times.
9) If everything is working, you can now close de box. 
11) If you have light leakeage trough the lens, add a black cardboard ring inside the LED ring. 

**Here are some pictures from my camera** <p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177628962-0bc55667-9e42-4df5-b561-c19c39566cfd.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177629909-fe780ae5-4fff-4817-a7e8-bd28ac0e00a5.png" height="300" align="center"><br><br>
<img src="https://user-images.githubusercontent.com/48807586/177629924-51aee180-2a6a-44b6-892f-6906a22174c9.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630302-cd32b6c9-6b18-49d8-9d33-7db810586d98.png" height="300" align="center">
</p>
    
### Server

Read the instructions at [`./server`](/server/)

### Clients

- Connect each RPi to a camera. 
    > The system has a 1cm accuracy when using 4 cameras, but the number can be expanded to improve cover.
- Put a cooler over each RPi. 
    > The high FPS will heat to 80C and activate frequency capping if no ventilation is directly over the processor.
- Read the instructions at [`./client`](/client/)

**Here are some pictures from my clients** <p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630783-b98d915a-e8e6-4619-8d5b-ff4ae7bf9cec.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630812-ef0bdeef-2afe-4c6c-b8f9-d032e451bc46.png" height="300" align="center">
</p>

### Vehicle
- Cover at least 4 polystyrene spheres with the reflective adehesive
    * I find it easier to cut into 3mm x 2cm pieces and glue them side by side.
- Use wire or strings to keep the spheres on the vehicle structure
    * I put some hot glue where the wire enters the sphere just to guarantee.
    * When the vehicle is not going to somersault, double-sided tape is enough.
     **Put at least three collinear spheres and one at a 90 degree angle to this line** 
     
**Here are some pictures from my markers** <p align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630567-203f1129-dc2e-4f36-b9f5-4dd15c5e4c1d.png" height="300" align="center">
<img src="https://user-images.githubusercontent.com/48807586/177630590-ecf808d8-98f1-44f5-a3be-9fbb3b7a658d.png" height="300" align="center"><br><br>
<img src="https://user-images.githubusercontent.com/48807586/182978086-e1b57256-8fcf-48b9-9657-b740c90c1c19.png" height="300" align="center">
</p>
