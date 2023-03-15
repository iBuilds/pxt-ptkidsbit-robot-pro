/**
 * Functions are mapped to blocks using various macros
 * in comments starting with %. The most important macro
 * is "block", and it specifies that a block should be
 * generated for an **exported** function.
 */

let inputString = ""
let distance = 0
let ready = 0

enum Motor_Write {
    //% block="Left"
    Motor_Left,
    //% block="Right"
    Motor_Right
}

enum _Turn {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum _Spin {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Servo_Write {
    //% block="S0"
    S0,
    //% block="S1"
    S1
}

enum Button_Name {
    //% block="BUTTONA"
    BUTTONA = 0x10,
    //% block="BUTTONB"
    BUTTONB = 0x20,
    //% block="BUTTONC"
    BUTTONC = 0x30
}

enum Button_Status {
    //% block="Pressed"
    Pressed = 0x01,
    //% block="Released"
    Released = 0x00
}

enum Sensor {
    //% block="Front"
    Front,
    //% block="Back"
    Back,
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum LED_Color {
    //% block="Red"
    Red = 1,
    //% block="Green"
    Green = 2,
    //% block="Blue"
    Blue = 3,
    //% block="White"
    White = 4,
    //% block="Black"
    Black = 5,
}

enum LED_FB {
    //% block="Front"
    Front,
    //% block="Back"
    Back
}

enum LED_LR {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Sensor_Status {
    //% block="ON"
    ON = 1,
    //% block="OFF"
    OFF = 0
}

enum ADC_Read {
    //% block="ADC0"
    ADC0 = 0,
    //% block="ADC1"
    ADC1 = 1,
    //% block="ADC2"
    ADC2 = 2,
    //% block="ADC3"
    ADC3 = 3,
    //% block="ADC4"
    ADC4 = 4,
    //% block="ADC5"
    ADC5 = 5,
    //% block="ADC6"
    ADC6 = 6,
    //% block="ADC7"
    ADC7 = 7,
    //% block="ADC8"
    ADC8 = 8,
    //% block="ADC9"
    ADC9 = 9,
    //% block="ADC10"
    ADC10 = 10,
    //% block="ADC11"
    ADC11 = 11,
    //% block="ADC12"
    ADC12 = 12,
    //% block="ADC13"
    ADC13 = 13,
    //% block="ADC14"
    ADC14 = 14,
    //% block="ADC15"
    ADC15 = 15,
    //% block="ADC16"
    ADC16 = 16,
    //% block="ADC17"
    ADC17 = 17,
    //% block="ADC18"
    ADC18 = 18,
    //% block="ADC19"
    ADC19 = 19,
    //% block="ADC20"
    ADC20 = 20,
    //% block="ADC21"
    ADC21 = 21,
    //% block="ADC22"
    ADC22 = 22,
    //% block="ADC23"
    ADC23 = 23
}

enum Calibrate_Mode {
    //% block="MODE1"
    MODE1,
    //% block="MODE2"
    MODE2
}

enum Forward_Direction {
    //% block="Forward"
    Forward,
    //% block="Backward"
    Backward
}

enum Intersection {
    //% block="|\n\u2513\n|"
    Left,
    //% block="|\n\u2533\n|"
    Center,
    //% block="|\n\u250F\n|"
    Right
}

enum Turn_Line {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Turn_Sensor {
    //% block="Center"
    Center,
    //% block="ADC1"
    ADC1,
    //% block="ADC2"
    ADC2,
    //% block="ADC3"
    ADC3,
    //% block="ADC4"
    ADC4
}

enum Direction_Robot {
    //% block="Front"
    Front,
    //% block="Back"
    Back
}

enum Stop_Position {
    //% block="Front"
    Front,
    //% block="Center"
    Center,
    //% block="Back"
    Back
}

enum Align {
    //% block="Enable"
    enable = 1,
    //% block="Disable"
    disable = 0
}

enum Button_State {
    state_1 = 0x10,
    state_2 = 0x11,
    state_3 = 0x20,
    state_4 = 0x21,
    state_5 = 0x30,
    state_6 = 0x31
}
interface KV {
    key: Button_State;
    action: Action;
}

//% color="#48C9B0" icon="\u2707"
namespace PTKidsBITRobotPRO {
    let kbCallback: KV[] = []
    function sendDataSerial(data: string) {
        inputString = ""
        serial.setWriteLinePadding(0)
        serial.redirect(
            SerialPin.P1,
            SerialPin.P8,
            BaudRate.BaudRate115200
        )
        serial.writeLine(data)
    }

    //% group="Movement Control"
    /**
     * Stop all Motor
     */
    //% block="Motor Stop"
    export function motorStop(): void {
        sendDataSerial("MC,0,0")
        basic.pause(10)
        serial.redirectToUSB()
    }

    //% group="Movement Control"
    /**
     * Set Power Brake Motor
     */
    //% block="Set Power Brake %power"
    //% power.min=0 power.max=255
    //% power.defl=50
    export function setPowerBrake(power: number): void {
        sendDataSerial("SP," + power)
        basic.pause(10)
        serial.redirectToUSB()
        ready = 1
    }

    // //% group="Movement Control"
    // /**
    //  * Compensate Speed Motor Left and Motor Right
    //  */
    // //% block="Compensate Left %Motor_Left|Compensate Right %Motor_Right"
    // //% left.min=-255 left.max=255
    // //% right.min=-255 right.max=255
    // export function motorCompensate(left: number, right: number): void {
    //     sendDataSerial("CP," + left + "," + right)
    //     basic.pause(10)
    //     serial.redirectToUSB()
    // }

    //% group="Movement Control"
    /**
     * Spin the Robot to Left or Right. The degree is adjustable.
     */
    //% block="Spin %degree degree"
    //% degree.min=-360 degree.max=360
    //% degree.defl=45
    export function spinDegree(degree: number): void {
        sendDataSerial("TD," + degree)
        // serial.readLine()
        while (!(serial.readLine().includes("OK")))
        basic.pause(10)
        serial.redirectToUSB()
    }

    //% group="Movement Control"
    /**
     * Spin the Robot to Left or Right. The speed motor is adjustable between 0 to 100.
     */
    //% block="Spin %_Spin|Speed %Speed"
    //% speed.min=0 speed.max=255
    //% speed.defl=100
    export function spin(spin: _Spin, speed: number): void {
        if (spin == _Spin.Left) {
            sendDataSerial("MC," + -speed + "," + speed)
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (spin == _Spin.Right) {
            sendDataSerial("MC," + speed + "," + -speed)
            basic.pause(10)
            serial.redirectToUSB()
        }
    }

    //% group="Movement Control"
    /**
     * Turn the Robot to Left or Right. The speed motor is adjustable between 0 to 100.
     */
    //% block="Turn %_Turn|Speed %Speed"
    //% speed.min=0 speed.max=255
    //% speed.defl=100
    export function turn(turn: _Turn, speed: number): void {
        if (turn == _Turn.Left) {
            sendDataSerial("MC,0," + speed)
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (turn == _Turn.Right) {
            sendDataSerial("MC," + speed + ",0")
            basic.pause(10)
            serial.redirectToUSB()
        }
    }

    //% group="Movement Control"
    /**
     * Control motors speed both at the same time. The speed motors is adjustable between -100 to 100.
     */
    //% block="Motor Left %Motor_Left|Motor Right %Motor_Right"
    //% speed1.min=-255 speed1.max=255
    //% speed2.min=-255 speed2.max=255
    //% speed1.defl=100
    //% speed2.defl=100
    export function motorGo(speed1: number, speed2: number): void {
        sendDataSerial("MC," + speed1 + "," + speed2)
        basic.pause(10)
        serial.redirectToUSB()
    }

    //% group="Movement Control"
    /**
     * Control motor speed 1 channel. The speed motor is adjustable between -100 to 100.
     */
    //% block="Motor Write %Motor_Write|Speed %Speed"
    //% speed.min=-255 speed.max=255
    //% speed.defl=100
    export function motorWrite(motor: Motor_Write, speed: number): void {
        if (motor == Motor_Write.Motor_Left) {
            sendDataSerial("MC," + speed + ",0")
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (motor == Motor_Write.Motor_Right) {
            sendDataSerial("MC,0," + speed)
            basic.pause(10)
            serial.redirectToUSB()
        }
    }

    //% group="Servo Control"
    /**
     * Control Servo Motor 0 - 180 Degrees
     */
    //% block="Servo %Servo_Write|Degree %Degree"
    //% degree.min=0 degree.max=180
    //% degree.defl=90
    export function servoWrite(servo: Servo_Write, degree: number): void {
        if (servo == Servo_Write.S0) {
            sendDataSerial("SC,0," + degree)
            basic.pause(10)
            serial.redirectToUSB()
        }
        else if (servo == Servo_Write.S1) {
            sendDataSerial("SC,1," + degree)
            basic.pause(10)
            serial.redirectToUSB()
        }
    }

    //% group="Sound"
    /**
     * Play Sound on Buzzer
     */
    //% block="Tone $note|Duration %duration"
    //% note.min=0 note.max=2000
    //% duration.min=0 duration.max=5000
    //% note.defl=800
    //% duration.defl=100
    export function tone(note: number, duration: number): void {
        sendDataSerial("PS," + note + "," + duration)
        basic.pause(10)
        serial.redirectToUSB()
    }

    //% group="Sensor and ADC"
    /**
     * Runs the program when the button is pressed
     */
    //% block="When $button|is %status"
    export function buttonEvent(button: Button_Name, status: Button_Status, a: Action) {
        let state = button + status
        let item: KV = { key: state, action: a }
        kbCallback.push(item)
    }

    // export function onButton(button: Button_Name, status: Button_Status): void {
    //     let timer = control.millis()
    //     while (1) {
    //         if (PTKidsBITRobotPRO.buttonRead(button) != status) timer = control.millis()
    //         if (control.millis() - timer > 110) break
    //     }
    // }

    //% group="Sensor and ADC"
    /**
     * Read Button Status
     */
    //% block="Button Read $button"
    export function buttonRead(button: Button_Name): number {
        let button_state = 0
        let button_name = 0
        if (button == Button_Name.BUTTONA) button_name = 0
        else if (button == Button_Name.BUTTONB) button_name = 1
        else if (button == Button_Name.BUTTONC) button_name = 2
        sendDataSerial("RB," + button_name)
        inputString = serial.readString()
        inputString = inputString.substr(0, inputString.length - 2)
        button_state = parseFloat(inputString)
        basic.pause(10)
        serial.redirectToUSB()
        return button_state
    }

    //% group="Sensor and ADC"
    /**
     * Read Analog from ADC Channel
     */
    //% block="ADC Read $pin"
    export function ADCRead(pin: ADC_Read): number {
        let adc_value = 0
        sendDataSerial("RS," + pin)
        inputString = serial.readString()
        inputString = inputString.substr(0, inputString.length - 2)
        adc_value = parseFloat(inputString)
        basic.pause(10)
        serial.redirectToUSB()
        return adc_value
    }

    //% group="Sensor and ADC"
    /**
     * Read Distance from Ultrasonic Sensor
     */
    //% block="GETDistance"
    export function distanceRead(): number {
        sendDataSerial("RD")
        inputString = serial.readString()
        inputString = inputString.substr(0, inputString.length - 2)
        if (parseFloat(inputString) >= 0) {
            distance = parseFloat(inputString)
        }
        basic.pause(30)
        serial.redirectToUSB()
        return distance
    }

    //% group="Sensor and ADC"
    /**
     * Turn on and turn off Line Sensor 
     */
    //% block="Sensor %sensor|Power $sensor_status"
    export function sensorPower(sensor: Sensor, sensor_status: Sensor_Status): void {
        let _sensor = ""
        if (sensor == Sensor.Front) _sensor = "f"
        else if (sensor == Sensor.Back) _sensor = "b"
        else if (sensor == Sensor.Left) _sensor = "l"
        else if (sensor == Sensor.Right) _sensor = "r"
        sendDataSerial("PL," + _sensor + "," + sensor_status)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Turn on and turn off Line Sensor All
     */
    //% block="Sensor Power $sensor_status"
    export function sensorPowerAll(sensor_status: Sensor_Status): void {
        sendDataSerial("PA," + sensor_status)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Set LED Brightness
     */
    //% block="Set LED Brightness %brightness"
    //% brightness.min=0 brightness.max=255
    //% brightness.defl=50
    export function setBrightness(brightness: number): void {
        sendDataSerial("SB," + brightness)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Set LED Color Left and Right
     */
    //% block="Set Color %LED_LR|Sensor 1  $color_1|Sensor 2  $color_2|Sensor 3  $color_3|Sensor 4  $color_4|Sensor 5  $color_5"
    export function setColorLR(sensor: LED_LR, color_1: LED_Color, color_2: LED_Color, color_3: LED_Color, color_4: LED_Color, color_5: LED_Color): void {
        let _sensor = ""
        if (sensor == LED_LR.Left) _sensor = "l"
        else if (sensor == LED_LR.Right) _sensor = "r"
        sendDataSerial("SL," + _sensor + "," + color_1 + "," + color_2 + "," + color_3 + "," + color_4 + "," + color_5)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Set LED Color Front and Black
     */
    //% block="Set Color %LED_FB|Sensor 1  $color_1|Sensor 2  $color_2|Sensor 3  $color_3|Sensor 4  $color_4|Sensor 5  $color_5|Sensor 6  $color_6|Sensor 7  $color_7"
    export function setColorFB(sensor: LED_FB, color_1: LED_Color, color_2: LED_Color, color_3: LED_Color, color_4: LED_Color, color_5: LED_Color, color_6: LED_Color, color_7: LED_Color): void {
        let _sensor = ""
        if (sensor == LED_FB.Front) _sensor = "f"
        else if (sensor == LED_FB.Back) _sensor = "b"
        sendDataSerial("SL," + _sensor + "," + color_1 + "," + color_2 + "," + color_3 + "," + color_4 + "," + color_5 + "," + color_6 + "," + color_7)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Sensor and ADC"
    /**
     * Set LED Color All
     */
    //% block="Set Color $color"
    export function setColor(color: LED_Color): void {
        sendDataSerial("SA," + color)
        basic.pause(10)
        serial.redirectToUSB()
        basic.pause(100)
    }

    //% group="Line Follower"
    /**
     * Turn Left or Right Follower Line Mode
     */
    //% block="Turn   %turn|Sensor %sensor|Align  %align|Speed\n %speed"
    //% speed.min=1 speed.max=3
    //% speed.defl=2
    //% align.defl=Align.disable
    export function TurnLINE(turn: Turn_Line, sensor: LED_FB, align: Align, speed: number) {
        let _direction = ""
        let _sensor = ""
        if (turn == Turn_Line.Left) _direction = "l"
        else if (turn == Turn_Line.Right) _direction = "r"
        if (sensor == LED_FB.Front) _sensor = "f"
        else if (sensor == LED_FB.Back) _sensor = "b"
        sendDataSerial("TL," + _direction + "," + _sensor + "," + speed + "," + align)
        // serial.readLine()
        while (!(serial.readLine().includes("OK")))
        basic.pause(10)
        serial.redirectToUSB()
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward Timer
     */
    //% block="Direction %direction|Time  %time|Speed %base_speed|KP %kp|KD %kd"
    //% speed.min=0 speed.max=255
    //% time.shadow="timePicker"
    //% speed.defl=180
    //% kp.defl=0.4
    //% kd.defl=4
    //% time.defl=500
    export function ForwardTIME(direction: Forward_Direction, time: number, speed: number, kp: number, kd: number) {
        if (direction == Forward_Direction.Forward) sendDataSerial("FT," + speed + "," + kp + "," + kd + "," + time)
        else if (direction == Forward_Direction.Backward) sendDataSerial("BT," + speed + "," + kp + "," + kd + "," + time)
        // serial.readLine()
        while (!(serial.readLine().includes("OK")))
        basic.pause(10)
        serial.redirectToUSB()
    }

    // //% group="Line Follower"
    // /**
    //  * Line Follower Forward find Custom Line
    //  */
    // //% block="Find %sensor|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    // //% sensor.defl="0--111"
    // //% min_speed.defl=30
    // //% max_speed.defl=100
    // //% kp.defl=0.01
    // //% min_speed.min=0 min_speed.max=100
    // //% max_speed.min=0 max_speed.max=100
    // export function ForwardLINECustom(sensor: string, min_speed: number, max_speed: number, kp: number, kd: number) {
        
    // }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Line
     */
    //% block="Junction   %Find_Line|Direction  %Forward_Direction|Stop Point %Stop_Position|Count %count|Speed %speed|KP %kp|KD %kd"
    //% find.defl=Intersection.Center
    //% stop_position.defl=Stop_Position.Center
    //% speed.min=0 speed.max=255
    //% count.defl=2
    //% speed.defl=70
    //% kp.defl=0.15
    //% kd.defl=1
    export function ForwardLINECount(find: Intersection, direction: Forward_Direction, stop_position: Stop_Position, count: number, speed: number, kp: number, kd: number) {
        let _intersection = ""
        let _stop_position = ""
        if (find == Intersection.Center) _intersection = "c"
        else if (find == Intersection.Left) _intersection = "l"
        else if (find == Intersection.Right) _intersection = "r"
        if (stop_position == Stop_Position.Front) _stop_position = "f"
        else if (stop_position == Stop_Position.Center) _stop_position = "c"
        else if (stop_position == Stop_Position.Back) _stop_position = "b"
        if (direction == Forward_Direction.Forward) sendDataSerial("FC," + speed + "," + kp + "," + kd + "," + _intersection + "," + _stop_position + "," + count)
        else if (direction == Forward_Direction.Backward) sendDataSerial("BC," + speed + "," + kp + "," + kd + "," + _intersection + "," + _stop_position + "," + count)
        // serial.readLine()
        while (!(serial.readLine().includes("OK")))
        basic.pause(10)
        serial.redirectToUSB()
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Line
     */
    //% block="Junction   %Find_Line|Direction  %Forward_Direction|Stop Point %Stop_Position|Speed %speed|KP %kp|KD %kd"
    //% find.defl=Intersection.Center
    //% stop_position.defl=Stop_Position.Center
    //% speed.min=0 speed.max=255
    //% speed.defl=70
    //% kp.defl=0.15
    //% kd.defl=1
    export function ForwardLINE(find: Intersection, direction: Forward_Direction, stop_position: Stop_Position, speed: number, kp: number, kd: number) {
        let _intersection = ""
        let _stop_position = ""
        if (find == Intersection.Center) _intersection = "c"
        else if (find == Intersection.Left) _intersection = "l"
        else if (find == Intersection.Right) _intersection = "r"
        if (stop_position == Stop_Position.Front) _stop_position = "f"
        else if (stop_position == Stop_Position.Center) _stop_position = "c"
        else if (stop_position == Stop_Position.Back) _stop_position = "b"
        if (direction == Forward_Direction.Forward) sendDataSerial("FL," + speed + "," + kp + "," + kd + "," + _intersection + "," + _stop_position)
        else if (direction == Forward_Direction.Backward) sendDataSerial("BL," + speed + "," + kp + "," + kd + "," + _intersection + "," + _stop_position)
        // serial.readLine()
        while (!(serial.readLine().includes("OK")))
        basic.pause(10)
        serial.redirectToUSB()
    }

    //% group="Line Follower"
    /**
     * Basic Line Follower
     */
    //% block="Direction %Forward_Direction|Speed %base_speed|KP %kp|KD %kd"
    //% speed.min=0 min_speed.max=255
    //% speed.defl=50
    //% kp.defl=0.12
    //% kd.defl=0.05
    export function Follower(direction: Forward_Direction, speed: number, kp: number, kd: number) {
        if (direction == Forward_Direction.Forward) sendDataSerial("FN," + speed + "," + kp + "," + kd)
        else if (direction == Forward_Direction.Backward) sendDataSerial("BN," + speed + "," + kp + "," + kd)
        basic.pause(5)
        serial.redirectToUSB()
    }

    //% group="Line Follower"
    /**
     * Read Position Line
     */
    //% block="Position Read $sensor"
    export function positionRead(sensor: Sensor): number {
        let _sensor = ""
        let position_value = 0
        if (sensor == Sensor.Front) _sensor = "f"
        else if (sensor == Sensor.Back) _sensor = "b"
        else if (sensor == Sensor.Left) _sensor = "l"
        else if (sensor == Sensor.Right) _sensor = "r"
        sendDataSerial("RP," + _sensor)
        inputString = serial.readString()
        inputString = inputString.substr(0, inputString.length - 2)
        position_value = parseFloat(inputString)
        basic.pause(10)
        serial.redirectToUSB()
        return position_value
    }

    //% group="Line Follower"
    /**
     * Calibrate Sensor
     */
    //% block="Set Calibrate Sensor $mode"
    export function SensorCalibrate(mode: Calibrate_Mode): void {
        if (mode == Calibrate_Mode.MODE1) {
            sendDataSerial("CS,1")
        }
        else if (mode == Calibrate_Mode.MODE2) {
            sendDataSerial("CS,2")
        }
        basic.pause(10)
        serial.redirectToUSB()
    }

    let x: number
    let i: number = 1
    function patorlState(): number {
        switch (i) {
            case 1: x = PTKidsBITRobotPRO.buttonRead(Button_Name.BUTTONA) == 0 ? 0x10 : 0; break;
            case 2: x = PTKidsBITRobotPRO.buttonRead(Button_Name.BUTTONA) == 1 ? 0x11 : 0; break;
            case 3: x = PTKidsBITRobotPRO.buttonRead(Button_Name.BUTTONB) == 0 ? 0x20 : 0; break;
            case 3: x = PTKidsBITRobotPRO.buttonRead(Button_Name.BUTTONB) == 1 ? 0x21 : 0; break;
            case 3: x = PTKidsBITRobotPRO.buttonRead(Button_Name.BUTTONC) == 0 ? 0x30 : 0; break;
            default: x = PTKidsBITRobotPRO.buttonRead(Button_Name.BUTTONC) == 1 ? 0x31 : 0; break;
        }
        i += 1;
        if (i == 7) i = 1
        return x;
    }

    basic.forever(() => {
        if (kbCallback != null && ready == 1) {
            let sta = patorlState()
            if (sta != 0) {
                for (let item of kbCallback) {
                    if (item.key == sta) {
                        item.action()
                    }
                }
            }
        }
        // basic.pause(50)
    })
}