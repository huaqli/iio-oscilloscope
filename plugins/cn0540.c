/**
 * Copyright (C) 2019 Analog Devices, Inc.
 *
 * Licensed under the GPL-2.
 *
 **/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <gtk/gtk.h>
#include <glib.h>
#include <math.h>

#include "../osc.h"
#include "../osc_plugin.h"
#include "../iio_widget.h"

#define THIS_DRIVER	"CN0540"
#define ADC_DEVICE	"ad7768-1"
#define DAC_DEVICE	"ltc2606"
#define GPIO_CTRL	"one-bit-adc-dac"
#define VOLTAGE_MONITOR_1   "xadc"
#define VOLTAGE_MONITOR_2   "ltc2308"
/*
 * For now leave this define as it is but, in future it would be nice if
 * the scaling is done in the driver so that, the plug in does not have
 * to knwow about  DAC_MAX_AMPLITUDE.
 */

#define DAC_SCALE               0.0625
#define ADC_SCALE               0.000488
#define G                       0.3
#define G_FDA                   2.667
#define V_OCM                   2.5
#define DAC_GAIN                1.22
#define AVG_COUNT               5

#define DAC_DEFAULT_VAL         55616

#define XADC_VREF               3.3
#define XADC_RES                12
#define NUM_GPIOS               8
#define NUM_ANALOG_PINS         6

struct iio_gpio {
    struct iio_channel *gpio;
    char label[30];
};
static struct iio_context *ctx;
static struct iio_channel *adc_ch;
static struct iio_channel *dac_ch;
static struct iio_channel *analog_in[NUM_ANALOG_PINS];
static struct iio_gpio gpio_ch[NUM_GPIOS];
static struct iio_widget iio_widgets[25];
static unsigned int num_widgets;

static GtkWidget *cn0540_panel;
static GtkRadioButton *radbtn_sw_ff;
static GtkCheckButton *tgbtn_shutdown;
static GtkCheckButton *tgbtn_fda;
static GtkCheckButton *tgbtn_fda_mode;
static GtkButton *btn_get_sw_ff;
static GtkTextView *sw_ff_status;
static GtkTextView *shutdown_status;
static GtkTextView *fda_status;
static GtkTextView *fda_mode_status;
static GtkTextView *voltage_0_status;
static GtkTextView *voltage_1_status;
static GtkTextView *voltage_2_status;
static GtkTextView *voltage_3_status;
static GtkTextView *voltage_4_status;
static GtkTextView *voltage_5_status;
static GtkTextBuffer *sw_ff_buffer;
static GtkTextBuffer *shutdown_buffer;
static GtkTextBuffer *fda_buffer;
static GtkTextBuffer *fda_mode_buffer;
static GtkTextBuffer *voltage_buffer[NUM_ANALOG_PINS];
static GtkWidget *calib_btn;
static GtkWidget *write_btn;
static GtkWidget *read_btn;
static GtkWidget *readvsensor_btn;
static GtkTextView *vshift_log;
static GtkTextView *vsensor_log;
static GtkTextView *calib_status;
static GtkTextBuffer *calib_buffer;
static GtkTextBuffer *vsensor_buf;
static GtkTextBuffer *vshift_buf;

static gboolean found_voltage_mon = FALSE;
static gboolean plugin_detached;
static gint this_page;

static gboolean cn0540_get_gpio_state(const char* gpio_name)
{
	long long readback = -1;

	for(int idx = 0; idx < NUM_GPIOS; idx++){
		if(strstr(gpio_ch[idx].label, gpio_name))
		iio_channel_attr_read_longlong(gpio_ch[idx].gpio, "raw",
					       &readback);
	}

	if(readback == -1)
		printf("%s: wrong gpio name: %s\n",__func__, gpio_name);

	return (gboolean)readback;
}

static void cn0540_set_gpio_state(const char* gpio_name, gboolean state)
{
	for(int idx = 0; idx < NUM_GPIOS; idx++){
        if (strstr(gpio_ch[idx].label, gpio_name))
            iio_channel_attr_write_longlong(gpio_ch[idx].gpio, "raw",
                                            (long long)state);
	}
}

static void monitor_shutdown(GtkCheckButton *btn)
{
	cn0540_set_gpio_state("cn0540_set_gpio_state",
			      btn->toggle_button.active);
	gtk_text_buffer_set_text(shutdown_buffer, btn->toggle_button.active ?
				 "ENABLED" : "DISABLED", -1);
}

static void monitor_sw_ff(GtkButton *btn)
{
	gboolean state;

	state = cn0540_get_gpio_state("cn0540_sw_ff_gpio");
	radbtn_sw_ff->check_button.toggle_button.active = state;
	gtk_text_buffer_set_text(sw_ff_buffer, state ? "HIGH" : "LOW", -1);
}

static void monitor_fda(GtkCheckButton *btn)
{
	cn0540_set_gpio_state("cn0540_FDA_DIS",!btn->toggle_button.active);
	gtk_text_buffer_set_text(fda_buffer, btn->toggle_button.active ?
				 "ENABLED" : "DISABLED", -1);
}

static void monitor_fda_mode(GtkCheckButton *btn)
{
	cn0540_set_gpio_state("cn0540_FDA_MODE",btn->toggle_button.active);
	gtk_text_buffer_set_text(fda_mode_buffer, btn->toggle_button.active ?
				 "FULL POWER" : "LOW POWER", -1);
}

static gboolean update_voltages(void)
{
	double scale, result;
	char voltage[10];
	long long raw;
	int idx;

	if (found_voltage_mon){
		for(idx = 0; idx < NUM_ANALOG_PINS; idx++) {
			iio_channel_attr_read_longlong(analog_in[idx], "raw",
						       &raw);
			iio_channel_attr_read_double(analog_in[idx], "scale",
			 			     &scale);
			result = raw * scale * XADC_VREF;
			snprintf(voltage, sizeof(voltage), "%.2f", result);
			gtk_text_buffer_set_text(voltage_buffer[idx], voltage,
						 -1);
		}
	}

	return TRUE;
}

static double vout_function(int32_t adc_code)
{
	double vout = (adc_code * ADC_SCALE) / 1000.0;
	return vout;

}
static double compute_vshift_cal(double vsensor)
{
	double vshift = (V_OCM +(G*vsensor))/(G+1);
	return vshift;
}

static double dac_code_to_v(uint16_t code)
{
	double ret = ((code*DAC_SCALE)/1000.0) * DAC_GAIN;
	return ret;
}
static uint16_t v_to_dac_code(double v)
{
	uint16_t ret = (uint16_t)(((v*1000.0)/DAC_SCALE)/DAC_GAIN);
	return ret;

}

static double vout_1st_stage(double vout)
{
	double ret = V_OCM - vout/G_FDA;
	return ret;

}
static double vsensor_function(double vout1st, double vshift)
{
	double vsensor = (((G+1)*vshift) - vout1st)/G;
	return vsensor;
}
static double get_vsensor_from_code(int32_t adc_code, uint16_t dac_code)
{
	double vout = vout_function(adc_code);
	double v1st = vout_1st_stage(vout);
	double vsensor = vsensor_function(v1st, dac_code_to_v(dac_code));
	return vsensor;
}
static void read_vshift(GtkButton *btn)
{
	char vshift_string[20];
	long long val;
	iio_channel_attr_read_longlong(dac_ch,"raw",&val);
	double vshift = dac_code_to_v(val);
	snprintf(vshift_string, sizeof(vshift_string), "%f", vshift);
	gtk_text_buffer_set_text(vshift_buf,vshift_string, -1);
}
static void write_vshift(GtkButton *btn)
{
	gchar *vshift_string;
	long long val;
	static GtkTextIter start, end;

	gtk_text_buffer_get_start_iter(vshift_buf,&start);
	gtk_text_buffer_get_end_iter(vshift_buf,&end);
	vshift_string = gtk_text_buffer_get_text(vshift_buf,&start,&end, -1);
	val = v_to_dac_code(atof(vshift_string));
	printf("val %lld\n",val);
	iio_channel_attr_write_longlong(dac_ch,"raw",val);
	g_free(vshift_string);
	fflush(stdout);
}

static long long read_adc_raw_val()
{
	long long val;
	long long sum = 0;
	int i;
	for(i = 0; i < AVG_COUNT;i++)
	{
		iio_channel_attr_read_longlong(adc_ch,"raw",&val);
		sum = sum + val;
	}
	val = (int32_t)(sum/AVG_COUNT);
	return val;
}
static void read_vsensor(GtkButton *btn)
{
	char vsensor_string[20];
	long long adc_code;
	long long dac_code;


	adc_code = read_adc_raw_val();
	iio_channel_attr_read_longlong(dac_ch,"raw",&dac_code);
	double vsensor = get_vsensor_from_code(adc_code,dac_code);
	snprintf(vsensor_string,sizeof(vsensor_string),"%f",vsensor);
	gtk_text_buffer_set_text(vsensor_buf,vsensor_string,-1);
}
static void calib(GtkButton *btn)
{
	static GtkTextIter iter;
	long long val;
	int32_t sum=0;
	char vshift_string[20], vsensor_string[20];
	int i;
	for(i = 0; i < AVG_COUNT;i++)
	{
		iio_channel_attr_read_longlong(adc_ch,"raw",&val);
		sum = sum + val;
	}
	val = (int32_t)(sum/AVG_COUNT);
	double vsensor = get_vsensor_from_code(val, DAC_DEFAULT_VAL);
	double vshift_cal = compute_vshift_cal(vsensor);
	uint16_t dac_code = v_to_dac_code(vshift_cal);
	iio_channel_attr_write_longlong(dac_ch,"raw",dac_code);
	iio_channel_attr_read_longlong(adc_ch, "raw", &val);
	vsensor = get_vsensor_from_code(val, dac_code);
	snprintf(vshift_string, sizeof(vshift_string), "%f", vshift_cal);
	snprintf(vsensor_string, sizeof(vsensor_string), "%f", vsensor);

	gtk_text_buffer_set_text(calib_buffer, "V(sensor)[V] --> ", -1);
	gtk_text_buffer_get_end_iter(calib_buffer, &iter);
	gtk_text_buffer_insert(calib_buffer,&iter,vsensor_string,-1);

	gtk_text_buffer_get_end_iter(calib_buffer, &iter);
	gtk_text_buffer_insert(calib_buffer, &iter,"Vshift[V] --> ", -1);
	gtk_text_buffer_get_end_iter(calib_buffer, &iter);
	gtk_text_buffer_insert(calib_buffer,&iter,vshift_string,-1);
}
static void save_widget_value(GtkWidget *widget, struct iio_widget *iio_w)
{
	iio_w->save(iio_w);
	/* refresh widgets so that, we know if our value was updated */
	iio_update_widgets(iio_widgets, num_widgets);
}

static void make_widget_update_signal_based(struct iio_widget *widgets,
					    unsigned int num_widgets)
{
	char signal_name[25];
	unsigned int i;

	for (i = 0; i < num_widgets; i++) {
		if (GTK_IS_CHECK_BUTTON(widgets[i].widget))
			sprintf(signal_name, "%s", "toggled");
		else if (GTK_IS_TOGGLE_BUTTON(widgets[i].widget))
			sprintf(signal_name, "%s", "toggled");
		else if (GTK_IS_SPIN_BUTTON(widgets[i].widget))
			sprintf(signal_name, "%s", "value-changed");
		else if (GTK_IS_COMBO_BOX_TEXT(widgets[i].widget))
			sprintf(signal_name, "%s", "changed");
		else
			printf("unhandled widget type, attribute: %s\n",
			       widgets[i].attr_name);

		if (GTK_IS_SPIN_BUTTON(widgets[i].widget) &&
				widgets[i].priv_progress != NULL) {
			iio_spin_button_progress_activate(&widgets[i]);
		} else {
			g_signal_connect(G_OBJECT(widgets[i].widget),
					 signal_name,
					 G_CALLBACK(save_widget_value),
					 &widgets[i]);
		}
	}
}

static GtkWidget *cn0540_init(struct osc_plugin *plugin, GtkWidget *notebook,
			      const char *ini_fn)
{
	GtkBuilder *builder;
	struct iio_device *adc;
	struct iio_device *dac;
	struct iio_device *gpio;
	struct iio_device *voltage_mon;
	gboolean direction;
	char *label;
	int idx;

	builder = gtk_builder_new();

	ctx = osc_create_context();
	if (!ctx)
		return NULL;

	if (osc_load_glade_file(builder, "cn0540") < 0)
		return NULL;

	adc = iio_context_find_device(ctx, ADC_DEVICE);
	dac = iio_context_find_device(ctx, DAC_DEVICE);
	gpio = iio_context_find_device(ctx, GPIO_CTRL);
	voltage_mon = iio_context_find_device(ctx, VOLTAGE_MONITOR_1);
	if (!voltage_mon)
		voltage_mon = iio_context_find_device(ctx, VOLTAGE_MONITOR_2);

	if (!adc || !dac) {
		printf("Could not find expected iio devices\n");
		return NULL;
	}
	if(voltage_mon)
		found_voltage_mon = TRUE;

	adc_ch = iio_device_find_channel(adc, "voltage0", false);
	dac_ch = iio_device_find_channel(dac, "voltage0", true);
	label = strdup("voltage0");
	idx = -1;
	direction = TRUE;
	while(1) {
		gpio_ch[++idx].gpio = iio_device_find_channel(gpio, label, direction);
		if (gpio_ch[idx].gpio != NULL){
			iio_channel_attr_read(gpio_ch[idx].gpio, "label", gpio_ch[idx].label, 30);
			label[7]++;
		} else if (direction && (gpio_ch[idx].gpio == NULL)) {
			direction = !direction;
			label = strdup("voltage0");
			idx--;
		} else
			break;
	}
	if (found_voltage_mon){
		label = strdup("voltage9");
		for(idx = 0; idx < NUM_ANALOG_PINS; idx++) {
			analog_in[idx] = iio_device_find_channel(voltage_mon, label, FALSE);
			label[strlen(label) - 1]++;
			if (label[7] == ':')
				label = strdup("voltage10");
		}
	}

	iio_channel_attr_write_longlong(dac_ch, "raw", DAC_DEFAULT_VAL);

	cn0540_panel = GTK_WIDGET(gtk_builder_get_object(builder,
							 "cn0540_panel"));

	radbtn_sw_ff = GTK_RADIO_BUTTON(gtk_builder_get_object(builder,"radbtn_sw_ff"));
	tgbtn_shutdown = GTK_CHECK_BUTTON(gtk_builder_get_object(builder,"tgbtn_shutdown"));
	tgbtn_fda = GTK_CHECK_BUTTON(gtk_builder_get_object(builder,"tgbtn_fda"));
	tgbtn_fda_mode = GTK_CHECK_BUTTON(gtk_builder_get_object(builder,"tgbtn_fda_mode"));
	btn_get_sw_ff = GTK_BUTTON(gtk_builder_get_object(builder,"btn_get_sw_ff"));
	sw_ff_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"sw_ff_status"));
	shutdown_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"shutdown_status"));
	fda_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"fda_status"));
	fda_mode_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"fda_mode_status "));
	voltage_0_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"voltage_0_status"));
	voltage_1_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"voltage_1_status"));
	voltage_2_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"voltage_2_status"));
	voltage_3_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"voltage_3_status"));
	voltage_4_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"voltage_4_status"));
	voltage_5_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"voltage_5_status"));
	sw_ff_buffer = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"sw_ff_buffer"));
	shutdown_buffer = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"shutdown_buffer"));
	fda_buffer = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"fda_buffer"));
	fda_mode_buffer = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"fda_mode_buffer"));
	voltage_buffer[0] = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"voltage_0_buffer"));
	voltage_buffer[1] = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"voltage_1_buffer"));
	voltage_buffer[2] = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"voltage_2_buffer"));
	voltage_buffer[3] = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"voltage_3_buffer"));
	voltage_buffer[4] = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"voltage_4_buffer"));
	voltage_buffer[5] = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"voltage_5_buffer"));
	calib_btn = GTK_WIDGET(gtk_builder_get_object(builder,"calib_btn"));
	read_btn = GTK_WIDGET(gtk_builder_get_object(builder,"read_btn"));
	write_btn = GTK_WIDGET(gtk_builder_get_object(builder,"write_btn"));
	readvsensor_btn = GTK_WIDGET(gtk_builder_get_object(builder,"readvsensor_btn"));
	calib_status = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"calib_status"));
	calib_buffer = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"calib_buffer"));
	vshift_log = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"vshift_log"));
	vshift_buf = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"vshift_buf"));
	vsensor_log = GTK_TEXT_VIEW(gtk_builder_get_object(builder,"vsensor_log"));
	vsensor_buf = GTK_TEXT_BUFFER(gtk_builder_get_object(builder,"vsensor_buf"));


	iio_toggle_button_init_from_builder(&iio_widgets[num_widgets++],
			adc, NULL, "en", builder, "radbtn_sw_ff", 0);

	iio_toggle_button_init_from_builder(&iio_widgets[num_widgets++],
			adc, NULL, "en", builder, "tgbtn_shutdown", 0);

	iio_toggle_button_init_from_builder(&iio_widgets[num_widgets++],
			adc, NULL, "en", builder, "tgbtn_fda", 0);

	iio_toggle_button_init_from_builder(&iio_widgets[num_widgets++],
			adc, NULL, "en", builder, "tgbtn_fda_mode", 0);

	iio_button_init_from_builder(&iio_widgets[num_widgets++],
			adc,NULL,NULL,builder,"btn_get_sw_ff");

	iio_button_init_from_builder(&iio_widgets[num_widgets++],
			adc,NULL,NULL,builder,"calib_btn");

	iio_button_init_from_builder(&iio_widgets[num_widgets++],
			adc,NULL,NULL,builder,
			"readvsensor_btn");

	iio_button_init_from_builder(&iio_widgets[num_widgets++],
			dac,NULL,NULL,builder,
			"read_btn");

	iio_button_init_from_builder(&iio_widgets[num_widgets++],
			dac,NULL,NULL,builder,
			"write_btn");

	make_widget_update_signal_based(iio_widgets, num_widgets);
	iio_update_widgets(iio_widgets, num_widgets);


	g_signal_connect(G_OBJECT(tgbtn_shutdown), "toggled",
			 G_CALLBACK(monitor_shutdown), NULL);
	g_signal_connect(G_OBJECT(tgbtn_fda), "toggled",
			 G_CALLBACK(monitor_fda), NULL);
	g_signal_connect(G_OBJECT(tgbtn_fda_mode), "toggled",
			 G_CALLBACK(monitor_fda_mode), NULL);
	g_signal_connect(G_OBJECT(btn_get_sw_ff), "clicked",
			 G_CALLBACK(monitor_sw_ff), NULL);
	g_signal_connect(G_OBJECT(calib_btn),"clicked",
			 G_CALLBACK(calib),NULL);
	g_signal_connect(G_OBJECT(read_btn),"clicked",
			 G_CALLBACK(read_vshift),NULL);
	g_signal_connect(G_OBJECT(write_btn),"clicked",
			 G_CALLBACK(write_vshift),NULL);
	g_signal_connect(G_OBJECT(readvsensor_btn),"clicked",
			 G_CALLBACK(read_vsensor),NULL);

	gtk_toggle_button_set_active(&tgbtn_shutdown->toggle_button,FALSE);
	gtk_toggle_button_set_active(&tgbtn_fda->toggle_button,TRUE);
	gtk_toggle_button_set_active(&tgbtn_fda_mode->toggle_button,TRUE);
	gtk_toggle_button_toggled(&tgbtn_shutdown->toggle_button);
	gtk_toggle_button_toggled(&tgbtn_fda->toggle_button);
	gtk_toggle_button_toggled(&tgbtn_fda_mode->toggle_button);
	gtk_button_clicked(btn_get_sw_ff);

	if (found_voltage_mon){
		g_timeout_add_seconds(1, (GSourceFunc)update_voltages, NULL);
	}

	return cn0540_panel;
}

static void update_active_page(struct osc_plugin *plugin, gint active_page,
			       gboolean is_detached)
{
	this_page = active_page;
	plugin_detached = is_detached;
}

static void cn0540_get_preferred_size(const struct osc_plugin *plugin,
				      int *width, int *height)
{
	if (width)
		*width = 640;
	if (height)
		*height = 480;
}

static void context_destroy(struct osc_plugin *plugin, const char *ini_fn)
{
	g_source_remove_by_user_data(ctx);
	osc_destroy_context(ctx);
}

struct osc_plugin plugin;

static bool cn0540_identify(const struct osc_plugin *plugin)
{
	/* Use the OSC's IIO context just to detect the devices */
	struct iio_context *osc_ctx = get_context_from_osc();

	return !!iio_context_find_device(osc_ctx, ADC_DEVICE) &&
			!!iio_context_find_device(osc_ctx, DAC_DEVICE) &&
			!!iio_context_find_device(osc_ctx, GPIO_CTRL);
}

struct osc_plugin plugin = {
	.name = THIS_DRIVER,
	.identify = cn0540_identify,
	.init = cn0540_init,
	.update_active_page = update_active_page,
	.get_preferred_size = cn0540_get_preferred_size,
	.destroy = context_destroy,
};
