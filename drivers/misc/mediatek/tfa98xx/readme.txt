have reset pin DTS:
&i2cx{
	tfa98xx@34 {
		compatible = "nxp,tfa98xx";
		reg = <0x34>;
		have-reset-pin = <1>;
		pinctrl-names = "smartpa_default",
		"smartpa_rst_h",
		"smartpa_rst_l";	
		pinctrl-0 = <&smartpa_default>;
		pinctrl-1 = <&smartpa_rst_h>;
		pinctrl-2 = <&smartpa_rst_l>;
		status = "okay";
	};
};
&pio {
	smartpa_default: smartpa_default {
	};

	smartpa_rst_h: smartpa_rst_out_one {
		pins_cmd_dat {
			pins = <PINMUX_GPIO100__FUNC_GPIO100>;
			slew-rate = <1>;
			bias-disable;
			output-high;
		};
	};

	smartpa_rst_l: smartpa_rst_out_zero {
		pins_cmd_dat {
			pins = <PINMUX_GPIO100__FUNC_GPIO100>;
			slew-rate = <1>;
			bias-disable;
			output-low;
		};
	};
};

have no reset pin DTS:
&i2cx{
	tfa98xx@34 {
		compatible = "nxp,tfa98xx";
		reg = <0x34>;
		have-reset-pin = <0>;
		status = "okay";
	};
};