/**
 * Functions are mapped to blocks using various macros
 * in comments starting with %. The most important macro
 * is "block", and it specifies that a block should be
 * generated for an **exported** function.
 */

let Sensor_All_PIN = [6, 5, 4, 3, 2, 1, 0, 8, 14, 13, 12, 11, 10, 9]
let Sensor_PIN_Front = [5, 4, 3, 2, 1]
let Sensor_Left_Front = [6]
let Sensor_Right_Front = [0]
let Sensor_PIN_Back = [13, 12, 11, 10, 9]
let Sensor_Left_Back = [14]
let Sensor_Right_Back = [8]
let Sensor_Center_Left = 7
let Sensor_Center_Right = 15
let Num_Sensor = 5
let KP = 0.004
let KD = 0.04
let Max_Speed = 100

let Version = 1
let Read_Version = false
let IMU_Address = 0x68
let PCA = 0x40
let initI2C = false
let initLED = false
let SERVOS = 0x06
let Line_LOW_Front = [0, 0, 0, 0, 0, 0, 0, 0]
let Line_HIGH_Front = [0, 0, 0, 0, 0, 0, 0, 0]
let Line_LOW_Center = [0, 0]
let Line_HIGH_Center = [0, 0]
let Color_Line_All_Front: number[] = []
let Color_Background_All_Front: number[] = []
let Color_Line_Front: number[] = []
let Color_Background_Front: number[] = []
let Color_Line_Left_Front: number[] = []
let Color_Background_Left_Front: number[] = []
let Color_Line_Center_Left: number[] = []
let Color_Background_Center_Left: number[] = []
let Color_Line_Right_Front: number[] = []
let Color_Background_Right_Front: number[] = []
let Color_Line_Center_Right: number[] = []
let Color_Background_Center_Right: number[] = []
let Line_All_Front = [0, 0, 0, 0, 0, 0, 0]
let Line_LOW_Back = [0, 0, 0, 0, 0, 0, 0, 0]
let Line_HIGH_Back = [0, 0, 0, 0, 0, 0, 0, 0]
let Color_Line_All_Back: number[] = []
let Color_Background_All_Back: number[] = []
let Color_Line_Back: number[] = []
let Color_Background_Back: number[] = []
let Color_Line_Left_Back: number[] = []
let Color_Background_Left_Back: number[] = []
let Color_Line_Right_Back: number[] = []
let Color_Background_Right_Back: number[] = []
let Line_All_Back = [0, 0, 0, 0, 0, 0, 0]
let Line_Mode = 0
let Last_Position = 0
let Compensate_Left = 0
let Compensate_Right = 0
let error = 0
let P = 0
let D = 0
let previous_error = 0
let PD_Value = 0
let left_motor_speed = 0
let right_motor_speed = 0
let Servo_8_Enable = 0
let Servo_12_Enable = 0
let Servo_8_Degree = 0
let Servo_12_Degree = 0
let distance = 0
let timer = 0

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
    //% block="P8"
    P8,
    //% block="P12"
    P12
}

enum Button_Status {
    //% block="Pressed"
    Pressed,
    //% block="Released"
    Released
}

enum ADC_Read {
    //% block="0"
    ADC0 = 0x84,
    //% block="1"
    ADC1 = 0xC4,
    //% block="2"
    ADC2 = 0x94,
    //% block="3"
    ADC3 = 0xD4,
    //% block="4"
    ADC4 = 0xA4,
    //% block="5"
    ADC5 = 0xE4,
    //% block="6"
    ADC6 = 0xB4,
    //% block="7"
    ADC7 = 0xF4,
    //% block="8"
    ADC8 = 0x8C,
    //% block="9"
    ADC9 = 0xCC,
    //% block="10"
    ADC10 = 0x9C,
    //% block="11"
    ADC11 = 0xDC,
    //% block="12"
    ADC12 = 0xAC,
    //% block="13"
    ADC13 = 0xEC,
    //% block="14"
    ADC14 = 0xBC,
    //% block="15"
    ADC15 = 0xFC
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

enum NeoPixelColors {
    //% block=red
    Red = 0xFF0000,
    //% block=orange
    Orange = 0xFFA500,
    //% block=yellow
    Yellow = 0xFFFF00,
    //% block=green
    Green = 0x00FF00,
    //% block=blue
    Blue = 0x0000FF,
    //% block=indigo
    Indigo = 0x4b0082,
    //% block=violet
    Violet = 0x8a2be2,
    //% block=purple
    Purple = 0xFF00FF,
    //% block=white
    White = 0xFFFFFF,
    //% block=black
    Black = 0x000000
}

enum NeoPixelMode {
    //% block="RGB (GRB format)"
    RGB = 1,
    //% block="RGB+W"
    RGBW = 2,
    //% block="RGB (RGB format)"
    RGB_RGB = 3
}

//% color="#48C9B0" icon="\u2707"
namespace PTKidsBITRobotPRO {
    class Strip {
        buf: Buffer;
        pin: DigitalPin;
        // TODO: encode as bytes instead of 32bit
        brightness: number;
        start: number; // start offset in LED strip
        _length: number; // number of LEDs
        _mode: NeoPixelMode;
        _matrixWidth: number; // number of leds in a matrix - if any

        /**
         * Shows all LEDs to a given color (range 0-255 for r, g, b).
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_strip_color" block="%strip|show color %rgb=neopixel_colors"
        //% strip.defl=strip
        //% weight=85 blockGap=8
        //% parts="neopixel"
        showColor(rgb: number) {
            rgb = rgb >> 0;
            this.setAllRGB(rgb);
            this.show();
        }

        /**
         * Shows a rainbow pattern on all LEDs.
         * @param startHue the start hue value for the rainbow, eg: 1
         * @param endHue the end hue value for the rainbow, eg: 360
         */
        //% blockId="neopixel_set_strip_rainbow" block="%strip|show rainbow from %startHue|to %endHue"
        //% strip.defl=strip
        //% weight=85 blockGap=8
        //% parts="neopixel"
        showRainbow(startHue: number = 1, endHue: number = 360) {
            if (this._length <= 0) return;
            startHue = startHue >> 0;
            endHue = endHue >> 0;
            const saturation = 100;
            const luminance = 50;
            const steps = this._length;
            const direction = HueInterpolationDirection.Clockwise;

            //hue
            const h1 = startHue;
            const h2 = endHue;
            const hDistCW = ((h2 + 360) - h1) % 360;
            const hStepCW = Math.idiv((hDistCW * 100), steps);
            const hDistCCW = ((h1 + 360) - h2) % 360;
            const hStepCCW = Math.idiv(-(hDistCCW * 100), steps);
            let hStep: number;
            if (direction === HueInterpolationDirection.Clockwise) {
                hStep = hStepCW;
            } else if (direction === HueInterpolationDirection.CounterClockwise) {
                hStep = hStepCCW;
            } else {
                hStep = hDistCW < hDistCCW ? hStepCW : hStepCCW;
            }
            const h1_100 = h1 * 100; //we multiply by 100 so we keep more accurate results while doing interpolation

            //sat
            const s1 = saturation;
            const s2 = saturation;
            const sDist = s2 - s1;
            const sStep = Math.idiv(sDist, steps);
            const s1_100 = s1 * 100;

            //lum
            const l1 = luminance;
            const l2 = luminance;
            const lDist = l2 - l1;
            const lStep = Math.idiv(lDist, steps);
            const l1_100 = l1 * 100

            //interpolate
            if (steps === 1) {
                this.setPixelColor(0, hsl(h1 + hStep, s1 + sStep, l1 + lStep))
            } else {
                this.setPixelColor(0, hsl(startHue, saturation, luminance));
                for (let i = 1; i < steps - 1; i++) {
                    const h = Math.idiv((h1_100 + i * hStep), 100) + 360;
                    const s = Math.idiv((s1_100 + i * sStep), 100);
                    const l = Math.idiv((l1_100 + i * lStep), 100);
                    this.setPixelColor(i, hsl(h, s, l));
                }
                this.setPixelColor(steps - 1, hsl(endHue, saturation, luminance));
            }
            this.show();
        }

        /**
         * Displays a vertical bar graph based on the `value` and `high` value.
         * If `high` is 0, the chart gets adjusted automatically.
         * @param value current value to plot
         * @param high maximum value, eg: 255
         */
        //% weight=84
        //% blockId=neopixel_show_bar_graph block="%strip|show bar graph of %value|up to %high"
        //% strip.defl=strip
        //% icon="\uf080"
        //% parts="neopixel"
        showBarGraph(value: number, high: number): void {
            if (high <= 0) {
                this.clear();
                this.setPixelColor(0, NeoPixelColors.Yellow);
                this.show();
                return;
            }

            value = Math.abs(value);
            const n = this._length;
            const n1 = n - 1;
            let v = Math.idiv((value * n), high);
            if (v == 0) {
                this.setPixelColor(0, 0x666600);
                for (let i = 1; i < n; ++i)
                    this.setPixelColor(i, 0);
            } else {
                for (let i = 0; i < n; ++i) {
                    if (i <= v) {
                        const b = Math.idiv(i * 255, n1);
                        this.setPixelColor(i, rgb(b, 0, 255 - b));
                    }
                    else this.setPixelColor(i, 0);
                }
            }
            this.show();
        }

        /**
         * Set LED to a given color (range 0-255 for r, g, b).
         * You need to call ``show`` to make the changes visible.
         * @param pixeloffset position of the NeoPixel in the strip
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_pixel_color" block="%strip|set pixel color at %pixeloffset|to %rgb=neopixel_colors"
        //% strip.defl=strip
        //% blockGap=8
        //% weight=80
        //% parts="neopixel" advanced=true
        setPixelColor(pixeloffset: number, rgb: number): void {
            this.setPixelRGB(pixeloffset >> 0, rgb >> 0);
        }

        /**
         * Sets the number of pixels in a matrix shaped strip
         * @param width number of pixels in a row
         */
        //% blockId=neopixel_set_matrix_width block="%strip|set matrix width %width"
        //% strip.defl=strip
        //% blockGap=8
        //% weight=5
        //% parts="neopixel" advanced=true
        setMatrixWidth(width: number) {
            this._matrixWidth = Math.min(this._length, width >> 0);
        }

        /**
         * Set LED to a given color (range 0-255 for r, g, b) in a matrix shaped strip
         * You need to call ``show`` to make the changes visible.
         * @param x horizontal position
         * @param y horizontal position
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_matrix_color" block="%strip|set matrix color at x %x|y %y|to %rgb=neopixel_colors"
        //% strip.defl=strip
        //% weight=4
        //% parts="neopixel" advanced=true
        setMatrixColor(x: number, y: number, rgb: number) {
            if (this._matrixWidth <= 0) return; // not a matrix, ignore
            x = x >> 0;
            y = y >> 0;
            rgb = rgb >> 0;
            const cols = Math.idiv(this._length, this._matrixWidth);
            if (x < 0 || x >= this._matrixWidth || y < 0 || y >= cols) return;
            let i = x + y * this._matrixWidth;
            this.setPixelColor(i, rgb);
        }

        /**
         * For NeoPixels with RGB+W LEDs, set the white LED brightness. This only works for RGB+W NeoPixels.
         * @param pixeloffset position of the LED in the strip
         * @param white brightness of the white LED
         */
        //% blockId="neopixel_set_pixel_white" block="%strip|set pixel white LED at %pixeloffset|to %white"
        //% strip.defl=strip
        //% blockGap=8
        //% weight=80
        //% parts="neopixel" advanced=true
        setPixelWhiteLED(pixeloffset: number, white: number): void {
            if (this._mode === NeoPixelMode.RGBW) {
                this.setPixelW(pixeloffset >> 0, white >> 0);
            }
        }

        /**
         * Send all the changes to the strip.
         */
        //% blockId="neopixel_show" block="%strip|show" blockGap=8
        //% strip.defl=strip
        //% weight=79
        //% parts="neopixel"
        show() {
            // only supported in beta
            // ws2812b.setBufferMode(this.pin, this._mode);
            ws2812b.sendBuffer(this.buf, this.pin);
        }

        /**
         * Turn off all LEDs.
         * You need to call ``show`` to make the changes visible.
         */
        //% blockId="neopixel_clear" block="%strip|clear"
        //% strip.defl=strip
        //% weight=76
        //% parts="neopixel"
        clear(): void {
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            this.buf.fill(0, this.start * stride, this._length * stride);
        }

        /**
         * Gets the number of pixels declared on the strip
         */
        //% blockId="neopixel_length" block="%strip|length" blockGap=8
        //% strip.defl=strip
        //% weight=60 advanced=true
        length() {
            return this._length;
        }

        /**
         * Set the brightness of the strip. This flag only applies to future operation.
         * @param brightness a measure of LED brightness in 0-255. eg: 255
         */
        //% blockId="neopixel_set_brightness" block="%strip|set brightness %brightness" blockGap=8
        //% strip.defl=strip
        //% weight=59
        //% parts="neopixel" advanced=true
        setBrightness(brightness: number): void {
            this.brightness = brightness & 0xff;
        }

        /**
         * Apply brightness to current colors using a quadratic easing function.
         **/
        //% blockId="neopixel_each_brightness" block="%strip|ease brightness" blockGap=8
        //% strip.defl=strip
        //% weight=58
        //% parts="neopixel" advanced=true
        easeBrightness(): void {
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            const br = this.brightness;
            const buf = this.buf;
            const end = this.start + this._length;
            const mid = Math.idiv(this._length, 2);
            for (let i = this.start; i < end; ++i) {
                const k = i - this.start;
                const ledoffset = i * stride;
                const br = k > mid
                    ? Math.idiv(255 * (this._length - 1 - k) * (this._length - 1 - k), (mid * mid))
                    : Math.idiv(255 * k * k, (mid * mid));
                const r = (buf[ledoffset + 0] * br) >> 8; buf[ledoffset + 0] = r;
                const g = (buf[ledoffset + 1] * br) >> 8; buf[ledoffset + 1] = g;
                const b = (buf[ledoffset + 2] * br) >> 8; buf[ledoffset + 2] = b;
                if (stride == 4) {
                    const w = (buf[ledoffset + 3] * br) >> 8; buf[ledoffset + 3] = w;
                }
            }
        }

        /**
         * Create a range of LEDs.
         * @param start offset in the LED strip to start the range
         * @param length number of LEDs in the range. eg: 4
         */
        //% weight=89
        //% blockId="neopixel_range" block="%strip|range from %start|with %length|leds"
        //% strip.defl=strip
        //% parts="neopixel"
        //% blockSetVariable=range
        range(start: number, length: number): Strip {
            start = start >> 0;
            length = length >> 0;
            let strip = new Strip();
            strip.buf = this.buf;
            strip.pin = this.pin;
            strip.brightness = this.brightness;
            strip.start = this.start + Math.clamp(0, this._length - 1, start);
            strip._length = Math.clamp(0, this._length - (strip.start - this.start), length);
            strip._matrixWidth = 0;
            strip._mode = this._mode;
            return strip;
        }

        /**
         * Shift LEDs forward and clear with zeros.
         * You need to call ``show`` to make the changes visible.
         * @param offset number of pixels to shift forward, eg: 1
         */
        //% blockId="neopixel_shift" block="%strip|shift pixels by %offset" blockGap=8
        //% strip.defl=strip
        //% weight=40
        //% parts="neopixel"
        shift(offset: number = 1): void {
            offset = offset >> 0;
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            this.buf.shift(-offset * stride, this.start * stride, this._length * stride)
        }

        /**
         * Rotate LEDs forward.
         * You need to call ``show`` to make the changes visible.
         * @param offset number of pixels to rotate forward, eg: 1
         */
        //% blockId="neopixel_rotate" block="%strip|rotate pixels by %offset" blockGap=8
        //% strip.defl=strip
        //% weight=39
        //% parts="neopixel"
        rotate(offset: number = 1): void {
            offset = offset >> 0;
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            this.buf.rotate(-offset * stride, this.start * stride, this._length * stride)
        }

        /**
         * Set the pin where the neopixel is connected, defaults to P0.
         */
        //% weight=10
        //% parts="neopixel" advanced=true
        setPin(pin: DigitalPin): void {
            this.pin = pin;
            pins.digitalWritePin(this.pin, 0);
            // don't yield to avoid races on initialization
        }

        /**
         * Estimates the electrical current (mA) consumed by the current light configuration.
         */
        //% weight=9 blockId=neopixel_power block="%strip|power (mA)"
        //% strip.defl=strip
        //% advanced=true
        power(): number {
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            const end = this.start + this._length;
            let p = 0;
            for (let i = this.start; i < end; ++i) {
                const ledoffset = i * stride;
                for (let j = 0; j < stride; ++j) {
                    p += this.buf[i + j];
                }
            }
            return Math.idiv(this.length() * 7, 10) /* 0.7mA per neopixel */
                + Math.idiv(p * 480, 10000); /* rought approximation */
        }

        private setBufferRGB(offset: number, red: number, green: number, blue: number): void {
            if (this._mode === NeoPixelMode.RGB_RGB) {
                this.buf[offset + 0] = red;
                this.buf[offset + 1] = green;
            } else {
                this.buf[offset + 0] = green;
                this.buf[offset + 1] = red;
            }
            this.buf[offset + 2] = blue;
        }

        private setAllRGB(rgb: number) {
            let red = unpackR(rgb);
            let green = unpackG(rgb);
            let blue = unpackB(rgb);

            const br = this.brightness;
            if (br < 255) {
                red = (red * br) >> 8;
                green = (green * br) >> 8;
                blue = (blue * br) >> 8;
            }
            const end = this.start + this._length;
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            for (let i = this.start; i < end; ++i) {
                this.setBufferRGB(i * stride, red, green, blue)
            }
        }
        private setAllW(white: number) {
            if (this._mode !== NeoPixelMode.RGBW)
                return;

            let br = this.brightness;
            if (br < 255) {
                white = (white * br) >> 8;
            }
            let buf = this.buf;
            let end = this.start + this._length;
            for (let i = this.start; i < end; ++i) {
                let ledoffset = i * 4;
                buf[ledoffset + 3] = white;
            }
        }
        private setPixelRGB(pixeloffset: number, rgb: number): void {
            if (pixeloffset < 0
                || pixeloffset >= this._length)
                return;

            let stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            pixeloffset = (pixeloffset + this.start) * stride;

            let red = unpackR(rgb);
            let green = unpackG(rgb);
            let blue = unpackB(rgb);

            let br = this.brightness;
            if (br < 255) {
                red = (red * br) >> 8;
                green = (green * br) >> 8;
                blue = (blue * br) >> 8;
            }
            this.setBufferRGB(pixeloffset, red, green, blue)
        }
        private setPixelW(pixeloffset: number, white: number): void {
            if (this._mode !== NeoPixelMode.RGBW)
                return;

            if (pixeloffset < 0
                || pixeloffset >= this._length)
                return;

            pixeloffset = (pixeloffset + this.start) * 4;

            let br = this.brightness;
            if (br < 255) {
                white = (white * br) >> 8;
            }
            let buf = this.buf;
            buf[pixeloffset + 3] = white;
        }
    }

    /**
     * Create a new NeoPixel driver for `numleds` LEDs.
     * @param pin the pin where the neopixel is connected.
     * @param numleds number of leds in the strip, eg: 24,30,60,64
     */
    //% blockId="neopixel_create" block="NeoPixel at pin %pin|with %numleds|leds as %mode"
    //% weight=90 blockGap=8
    //% parts="neopixel"
    //% trackArgs=0,2
    //% blockSetVariable=strip
    function create(pin: DigitalPin, numleds: number, mode: NeoPixelMode): Strip {
        let strip = new Strip();
        let stride = mode === NeoPixelMode.RGBW ? 4 : 3;
        strip.buf = pins.createBuffer(numleds * stride);
        strip.start = 0;
        strip._length = numleds;
        strip._mode = mode || NeoPixelMode.RGB;
        strip._matrixWidth = 0;
        strip.setBrightness(128)
        strip.setPin(pin)
        return strip;
    }

    /**
     * Converts red, green, blue channels into a RGB color
     * @param red value of the red channel between 0 and 255. eg: 255
     * @param green value of the green channel between 0 and 255. eg: 255
     * @param blue value of the blue channel between 0 and 255. eg: 255
     */
    //% weight=1
    //% blockId="neopixel_rgb" block="red %red|green %green|blue %blue"
    //% advanced=true
    function rgb(red: number, green: number, blue: number): number {
        return packRGB(red, green, blue);
    }

    /**
     * Gets the RGB value of a known color
    */
    //% weight=2 blockGap=8
    //% blockId="neopixel_colors" block="%color"
    //% advanced=true
    function colors(color: NeoPixelColors): number {
        return color;
    }

    function packRGB(a: number, b: number, c: number): number {
        return ((a & 0xFF) << 16) | ((b & 0xFF) << 8) | (c & 0xFF);
    }
    function unpackR(rgb: number): number {
        let r = (rgb >> 16) & 0xFF;
        return r;
    }
    function unpackG(rgb: number): number {
        let g = (rgb >> 8) & 0xFF;
        return g;
    }
    function unpackB(rgb: number): number {
        let b = (rgb) & 0xFF;
        return b;
    }

    /**
     * Converts a hue saturation luminosity value into a RGB color
     * @param h hue from 0 to 360
     * @param s saturation from 0 to 99
     * @param l luminosity from 0 to 99
     */
    //% blockId=neopixelHSL block="hue %h|saturation %s|luminosity %l"
    function hsl(h: number, s: number, l: number): number {
        h = Math.round(h);
        s = Math.round(s);
        l = Math.round(l);

        h = h % 360;
        s = Math.clamp(0, 99, s);
        l = Math.clamp(0, 99, l);
        let c = Math.idiv((((100 - Math.abs(2 * l - 100)) * s) << 8), 10000); //chroma, [0,255]
        let h1 = Math.idiv(h, 60);//[0,6]
        let h2 = Math.idiv((h - h1 * 60) * 256, 60);//[0,255]
        let temp = Math.abs((((h1 % 2) << 8) + h2) - 256);
        let x = (c * (256 - (temp))) >> 8;//[0,255], second largest component of this color
        let r$: number;
        let g$: number;
        let b$: number;
        if (h1 == 0) {
            r$ = c; g$ = x; b$ = 0;
        } else if (h1 == 1) {
            r$ = x; g$ = c; b$ = 0;
        } else if (h1 == 2) {
            r$ = 0; g$ = c; b$ = x;
        } else if (h1 == 3) {
            r$ = 0; g$ = x; b$ = c;
        } else if (h1 == 4) {
            r$ = x; g$ = 0; b$ = c;
        } else if (h1 == 5) {
            r$ = c; g$ = 0; b$ = x;
        }
        let m = Math.idiv((Math.idiv((l * 2 << 8), 100) - c), 2);
        let r = r$ + m;
        let g = g$ + m;
        let b = b$ + m;
        return packRGB(r, g, b);
    }

    export enum HueInterpolationDirection {
        Clockwise,
        CounterClockwise,
        Shortest
    }

    function readAdcAll(sensor: Forward_Direction): void {
        let ADC_PIN = [
            ADC_Read.ADC0,
            ADC_Read.ADC1,
            ADC_Read.ADC2,
            ADC_Read.ADC3,
            ADC_Read.ADC4,
            ADC_Read.ADC5,
            ADC_Read.ADC6,
            ADC_Read.ADC7,
            ADC_Read.ADC8,
            ADC_Read.ADC9,
            ADC_Read.ADC10,
            ADC_Read.ADC11,
            ADC_Read.ADC12,
            ADC_Read.ADC13,
            ADC_Read.ADC14,
            ADC_Read.ADC15
        ]
        if (sensor == Forward_Direction.Forward) {
            for (let i = 0; i < 7; i++) {
                let Value_Sensor = 0;
                if (Line_Mode == 0) {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[i]]), Color_Line_All_Front[i], Color_Background_All_Front[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                    Line_All_Front[i] = Value_Sensor
                }
                else {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[i]]), Color_Background_All_Front[i], Color_Line_All_Front[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                    Line_All_Front[i] = Value_Sensor
                }
            }
        }
        else {
            for (let i = 0; i < 7; i++) {
                let Value_Sensor = 0;
                if (Line_Mode == 0) {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[i + 7]]), Color_Line_All_Back[i], Color_Background_All_Back[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                    Line_All_Back[i] = Value_Sensor
                }
                else {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[i + 7]]), Color_Background_All_Back[i], Color_Line_All_Back[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                    Line_All_Back[i] = Value_Sensor
                }
            }
        }
    }

    function initPCA(): void {
        let i2cData = pins.createBuffer(2)
        initI2C = true
        i2cData[0] = 0
        i2cData[1] = 0x10
        pins.i2cWriteBuffer(PCA, i2cData, false)

        i2cData[0] = 0xFE
        i2cData[1] = 101
        pins.i2cWriteBuffer(PCA, i2cData, false)

        i2cData[0] = 0
        i2cData[1] = 0x81
        pins.i2cWriteBuffer(PCA, i2cData, false)

        for (let servo = 0; servo < 16; servo++) {
            i2cData[0] = SERVOS + servo * 4 + 0
            i2cData[1] = 0x00

            i2cData[0] = SERVOS + servo * 4 + 1
            i2cData[1] = 0x00
            pins.i2cWriteBuffer(PCA, i2cData, false);
        }
    }

    function setServoPCA(servo: number, angle: number): void {
        if (initI2C == false) {
            initPCA()
        }
        let i2cData = pins.createBuffer(2)
        let start = 0
        let angle_input = pins.map(angle, 0, 180, -90, 90)
        angle = Math.max(Math.min(90, angle_input), -90)
        let stop = 369 + angle * 235 / 90
        i2cData[0] = SERVOS + servo * 4 + 2
        i2cData[1] = (stop & 0xff)
        pins.i2cWriteBuffer(PCA, i2cData, false)

        i2cData[0] = SERVOS + servo * 4 + 3
        i2cData[1] = (stop >> 8)
        pins.i2cWriteBuffer(PCA, i2cData, false)
    }

    function i2cRead(reg: number): number {
        pins.i2cWriteNumber(IMU_Address, reg, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(IMU_Address, NumberFormat.UInt8BE);;
    }

    function readData(reg: number) {
        let h = i2cRead(reg);
        let l = i2cRead(reg + 1);
        let value = (h << 8) + l;

        if (value >= 0x8000) {
            return -((65535 - value) + 1);
        }
        else {
            return value;
        }
    }

    //% group="Motor Control"
    /**
     * Stop all Motor
     */
    //% block="Motor Stop"
    export function motorStop(): void {
        pins.digitalWritePin(DigitalPin.P16, 0)
        pins.digitalWritePin(DigitalPin.P15, 0)
        pins.digitalWritePin(DigitalPin.P14, 0)
        pins.digitalWritePin(DigitalPin.P13, 0)
    }

    //% group="Motor Control"
    /**
     * Compensate Speed Motor Left and Motor Right
     */
    //% block="Compensate Left %Motor_Left|Compensate Right %Motor_Right"
    //% left.min=-100 left.max=100
    //% right.min=-100 right.max=100
    export function motorCompensate(left: number, right: number): void {
        Compensate_Left = left
        Compensate_Right = right
    }

    //% group="Motor Control"
    /**
     * Spin the Robot to Left or Right. The speed motor is adjustable between 0 to 100.
     */
    //% block="Spin %_Spin|Speed %Speed"
    //% speed.min=0 speed.max=100
    export function Spin(spin: _Spin, speed: number): void {
        if (spin == _Spin.Left) {
            motorGo(-speed, speed)
        }
        else if (spin == _Spin.Right) {
            motorGo(speed, -speed)
        }
    }

    //% group="Motor Control"
    /**
     * Turn the Robot to Left or Right. The speed motor is adjustable between 0 to 100.
     */
    //% block="Turn %_Turn|Speed %Speed"
    //% speed.min=0 speed.max=100
    export function Turn(turn: _Turn, speed: number): void {
        if (turn == _Turn.Left) {
            motorGo(0, speed)
        }
        else if (turn == _Turn.Right) {
            motorGo(speed, 0)
        }
    }

    //% group="Motor Control"
    /**
     * Control motors speed both at the same time. The speed motors is adjustable between -100 to 100.
     */
    //% block="Motor Left %Motor_Left|Motor Right %Motor_Right"
    //% speed1.min=-100 speed1.max=100
    //% speed2.min=-100 speed2.max=100
    export function motorGo(speed1: number, speed2: number): void {
        if (speed1 >= 0) {
            speed1 = speed1 + Compensate_Left
        }
        else {
            speed1 = speed1 - Compensate_Left
        }

        if (speed2 >= 0) {
            speed2 = speed2 + Compensate_Right
        }
        else {
            speed2 = speed2 - Compensate_Right
        }

        if (speed1 < -100) {
            speed1 = -100
        }
        else if (speed1 > 100) {
            speed1 = 100
        }
        if (speed2 < -100) {
            speed2 = -100
        }
        else if (speed2 > 100) {
            speed2 = 100
        }

        speed1 = pins.map(speed1, -100, 100, -1023, 1023)
        speed2 = pins.map(speed2, -100, 100, -1023, 1023)

        if (speed1 < 0) {
            pins.digitalWritePin(DigitalPin.P15, 0)
            pins.analogWritePin(AnalogPin.P16, -speed1)
            pins.analogSetPeriod(AnalogPin.P15, 5000)
            pins.analogSetPeriod(AnalogPin.P16, 5000)
        }
        else if (speed1 >= 0) {
            pins.digitalWritePin(DigitalPin.P16, 0)
            pins.analogWritePin(AnalogPin.P15, speed1)
            pins.analogSetPeriod(AnalogPin.P15, 5000)
            pins.analogSetPeriod(AnalogPin.P16, 5000)
        }
        
        if (speed2 < 0) {
            pins.digitalWritePin(DigitalPin.P14, 0)
            pins.analogWritePin(AnalogPin.P13, -speed2)
            pins.analogSetPeriod(AnalogPin.P13, 5000)
            pins.analogSetPeriod(AnalogPin.P14, 5000)
        }
        else if (speed2 >= 0) {
            pins.digitalWritePin(DigitalPin.P13, 0)
            pins.analogWritePin(AnalogPin.P14, speed2)
            pins.analogSetPeriod(AnalogPin.P13, 5000)
            pins.analogSetPeriod(AnalogPin.P14, 5000)
        }
    }

    //% group="Motor Control"
    /**
     * Control motor speed 1 channel. The speed motor is adjustable between -100 to 100.
     */
    //% block="motorWrite %Motor_Write|Speed %Speed"
    //% speed.min=-100 speed.max=100
    export function motorWrite(motor: Motor_Write, speed: number): void {
        if (motor == Motor_Write.Motor_Left) {
            motorGo(speed, 0)
        }
        else if (motor == Motor_Write.Motor_Right) {
            motorGo(0, speed)
        }
    }

    //% group="RGB LED Control"
    /**
     * Set LED Color RGB
     */
    //% block="SETColor Red %red|Green %green|Blue\n %blue|Brightness %brightness"
    export function setColorRGB(red: number, green: number, blue: number, brightness: number): void {
        if (Read_Version == false) {
            let i2cData = pins.createBuffer(2)
            i2cData[0] = 0
            i2cData[1] = 16
            if (pins.i2cWriteBuffer(64, i2cData, false) == 0) {
                Version = 2
            }
            else {
                Version = 1
            }
            Read_Version = true
        }
        if (Version == 2) {
            let strip = create(DigitalPin.P8, 1, NeoPixelMode.RGB)
            strip.setBrightness(brightness)
            strip.showColor(rgb(red, green, blue))
        }
    }

    //% group="RGB LED Control"
    /**
     * Set LED Color
     */
    //% block="SETColor %colors|Brightness %brightness"
    export function setColor(color: NeoPixelColors, brightness: number): void {
        if (Read_Version == false) {
            let i2cData = pins.createBuffer(2)
            i2cData[0] = 0
            i2cData[1] = 16
            if (pins.i2cWriteBuffer(64, i2cData, false) == 0) {
                Version = 2
            }
            else {
                Version = 1
            }
            Read_Version = true
        }
        if (Version == 2) {
            let strip = create(DigitalPin.P8, 1, NeoPixelMode.RGB)
            strip.setBrightness(brightness)
            strip.showColor(colors(color))
        }
    }

    //% group="Servo Control"
    /**
     * Control Servo Motor 0 - 180 Degrees
     */
    //% block="Servo %Servo_Write|Degree %Degree"
    //% degree.min=0 degree.max=180
    export function servoWrite(servo: Servo_Write, degree: number): void {
        if (Read_Version == false) {
            let i2cData = pins.createBuffer(2)
            i2cData[0] = 0
            i2cData[1] = 16
            if (pins.i2cWriteBuffer(64, i2cData, false) == 0) {
                Version = 2
            }
            else {
                Version = 1
            }
            Read_Version = true
        }
        if (servo == Servo_Write.P8) {
            if (Version == 1) {
                Servo_8_Enable = 1
                Servo_8_Degree = degree
                pins.servoWritePin(AnalogPin.P8, Servo_8_Degree)
                pins.servoWritePin(AnalogPin.P12, Servo_12_Degree)
                basic.pause(100)
            }
            else {
                setServoPCA(1, degree)
            }
        }
        else if (servo == Servo_Write.P12) {
            if (Version == 1) {
                Servo_12_Enable = 1
                Servo_12_Degree = degree
                pins.servoWritePin(AnalogPin.P8, Servo_8_Degree)
                pins.servoWritePin(AnalogPin.P12, Servo_12_Degree)
                basic.pause(100)
            }
            else {
                setServoPCA(0, degree)
            }
        }
    }

    //% group="Sensor and ADC"
    /**
     * Read Robot Angle from MPU6050
     */
    //% block="GyroRead %ADC_Read"
    export function IMUAngleRead(): number {
        return readData(0x47) / 16.4;
    }

    //% group="Sensor and ADC"
    /**
     * Initialize MPU6050
     */
    //% block="IMUInitialize"
    export function initMPU6050() {
        let buffer = pins.createBuffer(2);
        buffer[0] = 0x6b;
        buffer[1] = 0;
        pins.i2cWriteBuffer(IMU_Address, buffer);
    }

    //% group="Sensor and ADC"
    /**
     * Read Analog from ADC Channel
     */
    //% block="ADCRead %ADC_Read"
    export function ADCRead(ADCRead: ADC_Read): number {
        if (ADCRead == ADC_Read.ADC0 || ADCRead == ADC_Read.ADC1 || ADCRead == ADC_Read.ADC2 || ADCRead == ADC_Read.ADC3 || ADCRead == ADC_Read.ADC4 || ADCRead == ADC_Read.ADC5 || ADCRead == ADC_Read.ADC6 || ADCRead == ADC_Read.ADC7) {
            pins.i2cWriteNumber(0x48, ADCRead, NumberFormat.UInt8LE, false)
            return ADCRead = pins.i2cReadNumber(0x48, NumberFormat.UInt16BE, false)
        }
        else {
            pins.i2cWriteNumber(0x49, ADCRead, NumberFormat.UInt8LE, false)
            return ADCRead = pins.i2cReadNumber(0x49, NumberFormat.UInt16BE, false)
        }
    }

    //% group="Sensor and ADC"
    /**
     * Read Distance from Ultrasonic Sensor
     */
    //% block="GETDistance"
    export function distanceRead(maxCmDistance = 500): number {
        let duration

        if (control.millis() - timer > 1000) {
            pins.setPull(DigitalPin.P1, PinPullMode.PullNone)
            pins.digitalWritePin(DigitalPin.P1, 0)
            control.waitMicros(2)
            pins.digitalWritePin(DigitalPin.P1, 1)
            control.waitMicros(10)
            pins.digitalWritePin(DigitalPin.P1, 0)
            duration = pins.pulseIn(DigitalPin.P2, PulseValue.High, maxCmDistance * 58)
            distance = Math.idiv(duration, 58)
        }

        pins.setPull(DigitalPin.P1, PinPullMode.PullNone)
        pins.digitalWritePin(DigitalPin.P1, 0)
        control.waitMicros(2)
        pins.digitalWritePin(DigitalPin.P1, 1)
        control.waitMicros(10)
        pins.digitalWritePin(DigitalPin.P1, 0)
        duration = pins.pulseIn(DigitalPin.P2, PulseValue.High, maxCmDistance * 58)
        let d = Math.idiv(duration, 58)

        if (d != 0) {
            distance = (0.1 * d) + (1 - 0.1) * distance
        }
        timer = control.millis()
        return Math.round(distance)
    }

    //% group="Line Follower"
    /**
     * Turn Left or Right Follower Line Mode
     */
    //% block="TurnLINE %turn|Speed\n %speed|Sensor %sensor|Fast Time %time"
    //% speed.min=0 speed.max=100
    //% time.shadow="timePicker"
    //% break_delay.shadow="timePicker"
    //% time.defl=200
    export function TurnLINE(turn: Turn_Line, speed: number, sensor: Turn_Sensor, time: number) {
        let ADC_PIN = [
            ADC_Read.ADC0,
            ADC_Read.ADC1,
            ADC_Read.ADC2,
            ADC_Read.ADC3,
            ADC_Read.ADC4,
            ADC_Read.ADC5,
            ADC_Read.ADC6,
            ADC_Read.ADC7,
            ADC_Read.ADC8,
            ADC_Read.ADC9,
            ADC_Read.ADC10,
            ADC_Read.ADC11,
            ADC_Read.ADC12,
            ADC_Read.ADC13,
            ADC_Read.ADC14,
            ADC_Read.ADC15
        ]
        let on_line = 0
        let adc_sensor_pin = sensor - 1
        let error = 0
        let motor_speed = 0
        let motor_slow = 20
        let timer = control.millis()
        let _position = 0
        let _position_min = 0
        let _position_max = 0

        while (1) {
            on_line = 0
            for (let i = 0; i < Sensor_PIN_Front.length; i++) {
                if ((pins.map(ADCRead(ADC_PIN[Sensor_PIN_Front[i]]), Color_Line_All_Front[i], Color_Background_All_Front[i], 1000, 0)) >= 500) {
                    on_line += 1;
                }
            }

            if (on_line == 0) {
                break
            }

            error = timer - (control.millis() - time)
            motor_speed = error

            if (motor_speed > speed) {
                motor_speed = speed
            }
            else if (motor_speed < 0) {
                motor_speed = motor_slow
            }

            if (turn == Turn_Line.Left) {
                motorGo(-motor_speed, motor_speed)
            }
            else if (turn == Turn_Line.Right) {
                motorGo(motor_speed, -motor_speed)
            }
        }
        while (1) {
            // if ((pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[adc_sensor_pin]]), Color_Line[adc_sensor_pin], Color_Background[adc_sensor_pin], 1000, 0)) >= 800) {
            //     motorStop()
            //     break
            // }
            if (sensor == Turn_Sensor.Center) {
                _position_min = 1250
                _position_max = 1750
            }
            else if (sensor == Turn_Sensor.ADC1) {
                _position_min = 2500
                _position_max = 3000
            }
            else if (sensor == Turn_Sensor.ADC2) {
                _position_min = 1500
                _position_max = 2000
            }
            else if (sensor == Turn_Sensor.ADC3) {
                _position_min = 1000
                _position_max = 1500
            }
            else if (sensor == Turn_Sensor.ADC4) {
                _position_min = 0
                _position_max = 500
            }

            _position = GETPosition(Direction_Robot.Front)
            if (_position > _position_min && _position < _position_max) {
                motorStop()
                break
            }
            else {
                error = timer - (control.millis() - time)
                motor_speed = error

                if (motor_speed > speed) {
                    motor_speed = speed
                }
                else if (motor_speed < 0) {
                    motor_speed = motor_slow
                }

                if (turn == Turn_Line.Left) {
                    motorGo(-motor_speed, motor_speed)
                }
                else if (turn == Turn_Line.Right) {
                    motorGo(motor_speed, -motor_speed)
                }
            }
        }
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward Timer
     */
    //% block="Time %time|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    //% time.shadow="timePicker"
    //% time.defl=200
    export function ForwardTIME(time: number, min_speed: number, max_speed: number, kp: number, kd: number) {
        let timer = control.millis()

        while (control.millis() - timer < time) {
            error = GETPosition(Direction_Robot.Back) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (kp * P) + (kd * D)
            previous_error = error

            left_motor_speed = min_speed + PD_Value
            right_motor_speed = min_speed - PD_Value

            if (left_motor_speed > max_speed) {
                left_motor_speed = max_speed
            }
            else if (left_motor_speed < -max_speed) {
                left_motor_speed = -max_speed
            }

            if (right_motor_speed > max_speed) {
                right_motor_speed = max_speed
            }
            else if (right_motor_speed < -max_speed) {
                right_motor_speed = -max_speed
            }

            motorGo(-left_motor_speed, -right_motor_speed)
        }
        motorStop()
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Custom Line
     */
    //% block="Find %sensor|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% sensor.defl="0--111"
    //% min_speed.defl=30
    //% max_speed.defl=100
    //% kp.defl=0.01
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    export function ForwardLINECustom(sensor: string, min_speed: number, max_speed: number, kp: number, kd: number) {
        let set_sensor = [0, 0, 0, 0, 0, 0]
        let on_line_setpoint = 500
        let out_line_setpoint = 100
        let sensor_interesting = 6
        for (let i = 0; i < 7; i ++) {
            if (sensor.charAt(i) == '-') {
                sensor_interesting -= 1
                set_sensor[i] = 2
            }
            else {
                set_sensor[i] = parseFloat(sensor.charAt(i))
            }
        }

        while (1) {
            let found = 0
            readAdcAll(Forward_Direction.Forward)

            for (let i = 0; i < 7; i++) {
                if (set_sensor[i] == 0) {
                    if (Line_All_Front[i] < out_line_setpoint) {
                        found += 1
                    }
                }
                else if (set_sensor[i] == 1) {
                    if (Line_All_Front[i] > on_line_setpoint) {
                        found += 1
                    }
                }
            }

            if (found >= sensor_interesting) {
                motorStop()
                break
            }

            error = GETPosition(Direction_Robot.Front) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (kp * P) + (kd * D)
            previous_error = error

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value

            if (left_motor_speed > max_speed) {
                left_motor_speed = max_speed
            }
            else if (left_motor_speed < -max_speed) {
                left_motor_speed = -max_speed
            }

            if (right_motor_speed > max_speed) {
                right_motor_speed = max_speed
            }
            else if (right_motor_speed < -max_speed) {
                right_motor_speed = -max_speed
            }

            motorGo(left_motor_speed, right_motor_speed)
        }
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Line
     */
    //% block="Junction\n\n %Intersection|Direction\n %Forward_Direction|Count Line %count|Base Speed %base_speed|Stop Point %Stop_Position"
    //% find.defl=Intersection.Center
    //% count.defl=1
    //% min_speed.defl=30
    //% min_speed.min=0 min_speed.max=100
    //% stop_position.defl=Stop_Position.Center
    export function ForwardLINECount(find: Intersection, Direction: Forward_Direction, count: number, min_speed: number, stop_position: Stop_Position) {
        let ADC_PIN = [
            ADC_Read.ADC0,
            ADC_Read.ADC1,
            ADC_Read.ADC2,
            ADC_Read.ADC3,
            ADC_Read.ADC4,
            ADC_Read.ADC5,
            ADC_Read.ADC6,
            ADC_Read.ADC7,
            ADC_Read.ADC8,
            ADC_Read.ADC9,
            ADC_Read.ADC10,
            ADC_Read.ADC11,
            ADC_Read.ADC12,
            ADC_Read.ADC13,
            ADC_Read.ADC14,
            ADC_Read.ADC15
        ]
        let break_direction = 0
        let on_line_setpoint = 900
        let _count = 0
        let _last_count = 0

        if (Direction == Forward_Direction.Forward) {
            break_direction = -1
        }
        else {
            break_direction = 1
        }

        while (1) {
            let found = 0
            readAdcAll(Direction)
            for (let i = 0; i < 7; i++) {
                if (Direction == Forward_Direction.Forward) {
                    if (Line_All_Front[i] > on_line_setpoint) {
                        found += 1
                    }
                }
                else {
                    if (Line_All_Back[i] > on_line_setpoint) {
                        found += 1
                    }
                }
            }

            if (find == Intersection.Center) {
                if (found >= 7) {
                    _count += 1
                    if (_count >= count) {
                        motorGo(min_speed * break_direction, min_speed * break_direction)
                        basic.pause(30)
                        while (1) {
                            let _sensor_left = pins.map(ADCRead(ADC_PIN[Sensor_Center_Left]), Color_Line_Center_Left[0], Color_Background_Center_Left[0], 1000, 0)
                            let _sensor_right = pins.map(ADCRead(ADC_PIN[Sensor_Center_Right]), Color_Line_Center_Right[0], Color_Background_Center_Right[0], 1000, 0)
                            if (_sensor_left >= 500 || _sensor_right >= 500) {
                                motorGo(10 * break_direction, 10 * break_direction)
                                basic.pause(50)
                                motorStop()
                                break
                            }
                            else {
                                motorGo(10 * -break_direction, 10 * -break_direction)
                            }
                        }
                        break
                    }
                    else {
                        while (1) {
                            let found = 0
                            readAdcAll(Direction)
                            for (let i = 0; i < 7; i++) {
                                if (Direction == Forward_Direction.Forward) {
                                    if (Line_All_Front[i] > on_line_setpoint) {
                                        found += 1
                                    }
                                }
                                else {
                                    if (Line_All_Back[i] > on_line_setpoint) {
                                        found += 1
                                    }
                                }
                            }

                            if (found >= 5) {
                                motorGo(min_speed * -break_direction, min_speed * -break_direction)
                            }
                            else {
                                break
                            }
                        }
                    }
                }
            }
            else {
                if (Direction == Forward_Direction.Forward) {
                    if (find == Intersection.Left) {
                        if (Line_All_Front[0] > on_line_setpoint && Line_All_Front[6] < 1000 - on_line_setpoint) {
                            _count += 1
                        }
                    }
                    else {
                        if (Line_All_Front[6] > on_line_setpoint && Line_All_Front[0] < 1000 - on_line_setpoint) {
                            _count += 1
                        }
                    }
                }
                else {
                    if (find == Intersection.Left) {
                        if (Line_All_Front[0] > on_line_setpoint && Line_All_Front[6] < 1000 - on_line_setpoint) {
                            _count += 1
                        }
                    }
                    else {
                        if (Line_All_Front[6] > on_line_setpoint && Line_All_Front[0] < 1000 - on_line_setpoint) {
                            _count += 1
                        }
                    }
                }

                if (_count != _last_count) {
                    motorGo(min_speed, min_speed)
                    while (1) {
                        let found = 0
                        readAdcAll(Direction)
                        for (let i = 0; i < 7; i++) {
                            if (Direction == Forward_Direction.Forward) {
                                if (Line_All_Front[i] > on_line_setpoint) {
                                    found += 1
                                }
                            }
                            else {
                                if (Line_All_Back[i] > on_line_setpoint) {
                                    found += 1
                                }
                            }
                        }
                        if (found >= 4) {
                            motorGo(min_speed * -break_direction, min_speed * -break_direction)
                        }
                        else {
                            _last_count = _count
                            break
                        }
                    }
                }

                if (_count >= count) {
                    motorGo(min_speed * break_direction, min_speed * break_direction)
                    basic.pause(20)
                    while (1) {
                        let _sensor_left = pins.map(ADCRead(ADC_PIN[Sensor_Center_Left]), Color_Line_Center_Left[0], Color_Background_Center_Left[0], 1000, 0)
                        let _sensor_right = pins.map(ADCRead(ADC_PIN[Sensor_Center_Right]), Color_Line_Center_Right[0], Color_Background_Center_Right[0], 1000, 0)
                        if (_sensor_left >= 500 || _sensor_right >= 500) {
                            motorGo(10 * break_direction, 10 * break_direction)
                            basic.pause(50)
                            motorStop()
                            break
                        }
                        else {
                            Follower(Direction, 10)
                        }
                    }
                    break
                }
            }
            Follower(Direction, min_speed)
        }
    }

    //% group="Line Follower"
    /**
     * Line Follower Forward find Line
     */
    //% block="Junction\n\n %Find_Line|Direction\n %Forward_Direction|Base Speed %base_speed|Stop Point %Stop_Position"
    //% find.defl=Intersection.Center
    //% min_speed.defl=30
    //% min_speed.min=0 min_speed.max=100
    //% stop_position.defl=Stop_Position.Center
    export function ForwardLINE(find: Intersection, Direction: Forward_Direction, min_speed: number, stop_position: Stop_Position) {
        let on_line_setpoint = 500

        while (1) {
            let found = 0
            readAdcAll(Forward_Direction.Forward)

            for (let i = 0; i < 7; i++) {
                if (Line_All_Front[i] > on_line_setpoint) {
                    found += 1
                }
            }

            if (found >= 5) {
                motorGo(min_speed, min_speed)
            }
            else {
                break
            }
        }

        while (1) {
            let found = 0
            readAdcAll(Forward_Direction.Forward)

            for (let i = 0; i < 7; i++) {
                if (Line_All_Front[i] > on_line_setpoint) {
                    found += 1
                }
            }

            if (find == Intersection.Center) {
                if (found >= 7) {
                    motorGo(-min_speed, -min_speed)
                    basic.pause(50)
                    motorStop()
                    break
                }
            }
            else if (find == Intersection.Left) {
                if (Line_All_Front[0] > on_line_setpoint && Line_All_Front[1] > on_line_setpoint && Line_All_Front[2] > on_line_setpoint && Line_All_Front[5] < 500) {
                    motorStop()
                    break
                }
            }
            else if (find == Intersection.Right) {
                if (Line_All_Front[3] > on_line_setpoint && Line_All_Front[4] > on_line_setpoint && Line_All_Front[5] > on_line_setpoint && Line_All_Front[0] < 500) {
                    motorStop()
                    break
                }
            }

            error = GETPosition(Direction_Robot.Front) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (KP * P) + (KD * D)
            previous_error = error

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value

            if (left_motor_speed > Max_Speed) {
                left_motor_speed = Max_Speed
            }
            else if (left_motor_speed < -Max_Speed) {
                left_motor_speed = -Max_Speed
            }

            if (right_motor_speed > Max_Speed) {
                right_motor_speed = Max_Speed
            }
            else if (right_motor_speed < -Max_Speed) {
                right_motor_speed = -Max_Speed
            }

            motorGo(left_motor_speed, right_motor_speed)
        }
    }

    //% group="Line Follower"
    /**
     * Basic Line Follower
     */
    //% block="Direction %Forward_Direction|Speed %base_speed"
    //% min_speed.defl=30
    //% min_speed.min=0 min_speed.max=100
    export function Follower(Direction: Forward_Direction, min_speed: number) {
        if (Direction == Forward_Direction.Forward) {
            error = GETPosition(Direction_Robot.Front) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            if (min_speed == 0) {
                PD_Value = (0.005 * P) + (0.001 * D)
                previous_error = error
            }
            else {
                PD_Value = (KP * P) + (KD * D)
                previous_error = error
            }

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value

            if (left_motor_speed > Max_Speed) {
                left_motor_speed = Max_Speed
            }
            else if (left_motor_speed < -Max_Speed) {
                left_motor_speed = -Max_Speed
            }

            if (right_motor_speed > Max_Speed) {
                right_motor_speed = Max_Speed
            }
            else if (right_motor_speed < -Max_Speed) {
                right_motor_speed = -Max_Speed
            }

            motorGo(left_motor_speed, right_motor_speed)
        }
        else {
            error = GETPosition(Direction_Robot.Back) - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (KP * P) + (KD * D)
            previous_error = error

            left_motor_speed = min_speed + PD_Value
            right_motor_speed = min_speed - PD_Value

            if (left_motor_speed > Max_Speed) {
                left_motor_speed = Max_Speed
            }
            else if (left_motor_speed < -Max_Speed) {
                left_motor_speed = -Max_Speed
            }

            if (right_motor_speed > Max_Speed) {
                right_motor_speed = Max_Speed
            }
            else if (right_motor_speed < -Max_Speed) {
                right_motor_speed = -Max_Speed
            }

            motorGo(-left_motor_speed, -right_motor_speed)
        }
    }

    //% group="Line Follower"
    /**
     * Get Position Line
     */
    //% block="GETPosition %Direction_Robot"
    export function GETPosition(Direction: Direction_Robot): number {
        let ADC_PIN = [
            ADC_Read.ADC0,
            ADC_Read.ADC1,
            ADC_Read.ADC2,
            ADC_Read.ADC3,
            ADC_Read.ADC4,
            ADC_Read.ADC5,
            ADC_Read.ADC6,
            ADC_Read.ADC7,
            ADC_Read.ADC8,
            ADC_Read.ADC9,
            ADC_Read.ADC10,
            ADC_Read.ADC11,
            ADC_Read.ADC12,
            ADC_Read.ADC13,
            ADC_Read.ADC14,
            ADC_Read.ADC15
        ]
        let Average = 0
        let Sum_Value = 0
        let ON_Line = 0

        if (Direction == Direction_Robot.Front) {
            for (let i = 0; i < Num_Sensor; i++) {
                let Value_Sensor = 0;
                if (Line_Mode == 0) {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Front[i]]), Color_Line_Front[i], Color_Background_Front[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                else {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Front[i]]), Color_Background_Front[i], Color_Line_Front[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                if (Value_Sensor > 200) {
                    ON_Line = 1;
                }
                Average += Value_Sensor * (i * 1000)
                Sum_Value += Value_Sensor
            }
        }
        else {
            for (let i = 0; i < Num_Sensor; i++) {
                let Value_Sensor = 0;
                if (Line_Mode == 0) {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Back[i]]), Color_Line_Back[i], Color_Background_Back[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                else {
                    Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN_Back[i]]), Color_Background_Back[i], Color_Line_Back[i], 1000, 0)
                    if (Value_Sensor < 0) {
                        Value_Sensor = 0
                    }
                    else if (Value_Sensor > 1000) {
                        Value_Sensor = 1000
                    }
                }
                if (Value_Sensor > 200) {
                    ON_Line = 1;
                }
                Average += Value_Sensor * (i * 1000)
                Sum_Value += Value_Sensor
            }
        }

        if (ON_Line == 0) {
            if (Last_Position < (Num_Sensor - 1) * 1000 / 2) {
                return (Num_Sensor - 1) * 1000
            }
            else {
                return 0
            }
        }
        Last_Position = Average / Sum_Value;
        return Math.round(((Num_Sensor - 1) * 1000) - Last_Position)
    }

    //% group="Line Follower"
    /**
     * Calibrate Sensor
     */
    //% block="SensorCalibrate $adc_pin"
    export function SensorCalibrate(): void {
        let ADC_PIN = [
            ADC_Read.ADC0,
            ADC_Read.ADC1,
            ADC_Read.ADC2,
            ADC_Read.ADC3,
            ADC_Read.ADC4,
            ADC_Read.ADC5,
            ADC_Read.ADC6,
            ADC_Read.ADC7,
            ADC_Read.ADC8,
            ADC_Read.ADC9,
            ADC_Read.ADC10,
            ADC_Read.ADC11,
            ADC_Read.ADC12,
            ADC_Read.ADC13,
            ADC_Read.ADC14,
            ADC_Read.ADC15
        ]
        let _Sensor_PIN_Front = [6, 5, 4, 3, 2, 1, 0]
        let _Sensor_PIN_Back = [14, 13, 12, 11, 10, 9, 8]
        let _Sensor_PIN_Center = [7, 15]
        let Line_Cal_Front = [0, 0, 0, 0, 0, 0, 0, 0]
        let Line_Cal_Back = [0, 0, 0, 0, 0, 0, 0, 0]
        let Line_Cal_Center = [0, 0]
        let Background_Cal_Front = [0, 0, 0, 0, 0, 0, 0, 0]
        let Background_Cal_Back = [0, 0, 0, 0, 0, 0, 0, 0]
        let Background_Cal_Center = [0, 0]
        let _Num_Sensor = 7

        basic.pause(300)
        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(587, music.beat(BeatFraction.Quarter))
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        ////Calibrate Follower Line
        while (!input.buttonIsPressed(Button.A));
        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        for (let i = 0; i < 20; i++) {
            for (let j = 0; j < _Num_Sensor; j++) {
                Line_Cal_Front[j] += ADCRead(ADC_PIN[_Sensor_PIN_Front[j]])
                Line_Cal_Back[j] += ADCRead(ADC_PIN[_Sensor_PIN_Back[j]])
            }
            Line_Cal_Center[0] += ADCRead(ADC_PIN[_Sensor_PIN_Center[0]])
            Line_Cal_Center[1] += ADCRead(ADC_PIN[_Sensor_PIN_Center[1]])
            basic.pause(50)
        }
        for (let i = 0; i < _Num_Sensor; i++) {
            Line_Cal_Front[i] = Line_Cal_Front[i] / 20
            Line_Cal_Back[i] = Line_Cal_Back[i] / 20
            for (let j = 0; j < 8; j++) {
                if (_Sensor_PIN_Front[i] == j) {
                    Line_HIGH_Front[j] = Line_Cal_Front[i]
                }
                if ((_Sensor_PIN_Back[i] - 8) == j) {
                    Line_HIGH_Back[j] = Line_Cal_Back[i]
                }
            }
        }
        Line_Cal_Center[0] = Line_Cal_Center[0] / 20
        Line_Cal_Center[1] = Line_Cal_Center[1] / 20
        Line_HIGH_Center[0] = Line_Cal_Center[0]
        Line_HIGH_Center[1] = Line_Cal_Center[1]

        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        ////Calibrate Background
        while (!input.buttonIsPressed(Button.A));
        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        for (let i = 0; i < 20; i++) {
            for (let j = 0; j < _Num_Sensor; j++) {
                Background_Cal_Front[j] += ADCRead(ADC_PIN[_Sensor_PIN_Front[j]])
                Background_Cal_Back[j] += ADCRead(ADC_PIN[_Sensor_PIN_Back[j]])
            }
            Background_Cal_Center[0] += ADCRead(ADC_PIN[_Sensor_PIN_Center[0]])
            Background_Cal_Center[1] += ADCRead(ADC_PIN[_Sensor_PIN_Center[1]])
            basic.pause(50)
        }
        for (let i = 0; i < _Num_Sensor; i++) {
            Background_Cal_Front[i] = Background_Cal_Front[i] / 20
            Background_Cal_Back[i] = Background_Cal_Back[i] / 20
            for (let j = 0; j < 8; j++) {
                if (_Sensor_PIN_Front[i] == j) {
                    Line_LOW_Front[j] = Background_Cal_Front[i]
                }
                if ((_Sensor_PIN_Back[i] - 8) == j) {
                    Line_LOW_Back[j] = Background_Cal_Back[i]
                }
            }
        }
        Background_Cal_Center[0] = Background_Cal_Center[0] / 20
        Background_Cal_Center[1] = Background_Cal_Center[1] / 20
        Line_LOW_Center[0] = Background_Cal_Center[0]
        Line_LOW_Center[1] = Background_Cal_Center[1]

        for (let i = 0; i < Num_Sensor; i++) {
            Color_Line_Front[i] = Line_HIGH_Front[Sensor_PIN_Front[i]]
            Color_Line_Back[i] = Line_HIGH_Back[Sensor_PIN_Back[i] - 8]
            Color_Background_Front[i] = Line_LOW_Front[Sensor_PIN_Front[i]]
            Color_Background_Back[i] = Line_LOW_Back[Sensor_PIN_Back[i] - 8]
        }
        for (let i = 0; i < Sensor_Left_Front.length; i++) {
            Color_Line_Left_Front[i] = Line_HIGH_Front[Sensor_Left_Front[i]]
            Color_Background_Left_Front[i] = Line_LOW_Front[Sensor_Left_Front[i]]
        }
        for (let i = 0; i < Sensor_Left_Back.length; i++) {
            Color_Line_Left_Back[i] = Line_HIGH_Back[Sensor_Left_Back[i] - 8]
            Color_Background_Left_Back[i] = Line_LOW_Back[Sensor_Left_Back[i] - 8]
        }
        for (let i = 0; i < Sensor_Right_Front.length; i++) {
            Color_Line_Right_Front[i] = Line_HIGH_Front[Sensor_Right_Front[i]]
            Color_Background_Right_Front[i] = Line_LOW_Front[Sensor_Right_Front[i]]
        }
        for (let i = 0; i < Sensor_Right_Back.length; i++) {
            Color_Line_Right_Back[i] = Line_HIGH_Back[Sensor_Right_Back[i] - 8]
            Color_Background_Right_Back[i] = Line_LOW_Back[Sensor_Right_Back[i] - 8]
        }

        Color_Line_Center_Left[0] = Line_HIGH_Center[0]
        Color_Background_Center_Left[0] = Line_LOW_Center[0]
        Color_Line_Center_Right[0] = Line_HIGH_Center[1]
        Color_Background_Center_Right[0] = Line_LOW_Center[1]

        Color_Line_All_Front = [Color_Line_Left_Front[0], Color_Line_Front[0], Color_Line_Front[1], Color_Line_Front[2], Color_Line_Front[3], Color_Line_Front[4], Color_Line_Right_Front[0]]
        Color_Line_All_Back = [Color_Line_Left_Back[0], Color_Line_Back[0], Color_Line_Back[1], Color_Line_Back[2], Color_Line_Back[3], Color_Line_Back[4], Color_Line_Right_Back[0]]
        Color_Background_All_Front = [Color_Background_Left_Front[0], Color_Background_Front[0], Color_Background_Front[1], Color_Background_Front[2], Color_Background_Front[3], Color_Background_Front[4], Color_Background_Right_Front[0]]
        Color_Background_All_Back = [Color_Background_Left_Back[0], Color_Background_Back[0], Color_Background_Back[1], Color_Background_Back[2], Color_Background_Back[3], Color_Background_Back[4], Color_Background_Right_Back[0]]

        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        music.playTone(587, music.beat(BeatFraction.Quarter))
        if (Servo_8_Enable == 1) {
            pins.servoWritePin(AnalogPin.P8, Servo_8_Degree)
        }
        if (Servo_12_Enable == 1) {
            pins.servoWritePin(AnalogPin.P12, Servo_12_Degree)
        }
        basic.pause(200)
    }
}