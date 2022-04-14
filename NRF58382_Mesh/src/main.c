/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/printk.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <device.h>
#include <drivers/gpio.h>

#define MOD_LF 0x0000

#define GROUP_ADDR 0xc000
#define PUBLISHER_ADDR  0x000f

#define OP_VENDOR_BUTTON BT_MESH_MODEL_OP_3(0x00, BT_COMP_ID_LF)
#define OP_KOS_MESSAGE BT_MESH_MODEL_OP_3(0xD0, BT_COMP_ID_LF)

static const uint8_t net_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint8_t dev_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint8_t app_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint16_t net_idx;
static const uint16_t app_idx;
static const uint32_t iv_index;
static uint8_t flags;
static uint16_t addr = NODE_ADDR;
static const struct device *gpio0;
int unlock_count = 0;
int lock_count = 0;

void LockOff(void);
void LockOn(void);
static void heartbeat(const struct bt_mesh_hb_sub *sub, uint8_t hops,
		      uint16_t feat)
{
	printk("|   NRF58382   | Got heartbeat\n");
}

static struct bt_mesh_cfg_cli cfg_cli = {
};

static void attention_on(struct bt_mesh_model *model)
{
	printk("|   NRF58382   | attention_on()\n");	
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("|   NRF58382   | attention_off()\n");
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
};

uint32_t unlock_prbs()
{   
    static uint32_t shift_register=0xa551199; // "random" seed value
	int b1 = 0;
	int b2 = 0;
	if (shift_register & (1 << 30))
	{
		b1 = 1;
	}
	if (shift_register & (1 << 27))
	{
		b2 = 1;
	}
	
	shift_register=shift_register << 1;
	shift_register=shift_register | (b1 ^ b2);
	shift_register = shift_register & 0x7fffffff;
    return shift_register ; // return 31 LSB's 
}



static int kos_message_received(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{	
    
    uint32_t unlock_code; 
    uint32_t data = net_buf_simple_pull_le32(buf);	//if message is too long it looks at the first address of the stack i think
    if(data != 20) {
	    printk("|   NRF58382   | Received KOS message from Microbit %x\n",data);
	    printk("|   NRF58382   | RSSI=%d\n",ctx->recv_rssi);        
	    
	    if (data == 10){
	    	printk("|   NRF58382   | Message Received from Microbit is a SYN\n");
	    	printk("|   NRF58382   | Incrementing PRBS Generator\n");
	    	unlock_code = unlock_prbs();
		unlock_code = unlock_code & 0xfffffff;
		printk("|   NRF58382   | PRBS CODE GENERATED %x\n",unlock_code);
	    	printk("|   NRF58382   | Sending Acknowledgement (ACK)\n");
	    	sendDMBMessage(20);
	    }
    
		if (data == unlock_code) //only matches when number is a decimal
		{
			printk("|   NRF58382   | Unlock Code Recieved: %x\n", data);
			printk("|   NRF58382   | Button A pressed\n");
			printk("|   NRF58382   | ITS A MATCH\n");
			LockOff();
		}
		if (data == 0xf109c5) //there is a chance that the codes will match at one point
		{
			printk("|   NRF58382   | Lock Code Recieved: %x\n", data);
			printk("|   NRF58382   | Button B pressed\n");
			LockOn();
		}
	
    }
	return 0;
}
static const struct bt_mesh_model_op vnd_ops[] = {	
	{ OP_KOS_MESSAGE, BT_MESH_LEN_EXACT(0), kos_message_received },
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model vnd_models[] = {
	BT_MESH_MODEL_VND(BT_COMP_ID_LF, MOD_LF, vnd_ops, NULL, NULL),
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, vnd_models),
};

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static void configure(void)
{
	printk("|   NRF58382   | Configuring...\n");

	/* Add Application Key */
	bt_mesh_cfg_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);

	/* Bind to vendor model */
	bt_mesh_cfg_mod_app_bind_vnd(net_idx, addr, addr, app_idx,
				     MOD_LF, BT_COMP_ID_LF, NULL);

	/* Bind to Health model */
	bt_mesh_cfg_mod_app_bind(net_idx, addr, addr, app_idx,
				 BT_MESH_MODEL_ID_HEALTH_SRV, NULL);

	/* Add model subscription */
	bt_mesh_cfg_mod_sub_add_vnd(net_idx, addr, addr, GROUP_ADDR,
				    MOD_LF, BT_COMP_ID_LF, NULL);

#if NODE_ADDR == PUBLISHER_ADDR
	{
		struct bt_mesh_cfg_hb_pub pub = {
			.dst = GROUP_ADDR,
			.count = 0xff,
			.period = 0x05,
			.ttl = 0x07,
			.feat = 0,
			.net_idx = net_idx,
		};

		bt_mesh_cfg_hb_pub_set(net_idx, addr, &pub, NULL);
		printk("|   NRF58382   | Publishing heartbeat messages\n");
	}
#endif
	printk("|   NRF58382   | Configuration complete\n");
}

static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
};

BT_MESH_HB_CB_DEFINE(hb_cb) = {
	.recv = heartbeat,
};

static void bt_ready(int err)
{
	if (err) {
		printk("|   NRF58382   | Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("|   NRF58382   | Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("|   NRF58382   | Initializing mesh failed (err %d)\n", err);
		return;
	}

	printk("|   NRF58382   | Mesh initialized\n");

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		printk("|   NRF58382   | Loading stored settings\n");
		settings_load();
	}

	err = bt_mesh_provision(net_key, net_idx, flags, iv_index, addr,
				dev_key);
	if (err == -EALREADY) {
		printk("|   NRF58382   | Using stored settings\n");
	} else if (err) {
		printk("|   NRF58382   | Provisioning failed (err %d)\n", err);
		return;
	} else {
		printk("|   NRF58382   | Provisioning completed\n");
		configure();
	}

#if NODE_ADDR != PUBLISHER_ADDR
	/* Heartbeat subcscription is a temporary state (due to there
	 * not being an "indefinite" value for the period, so it never
	 * gets stored persistently. Therefore, we always have to configure
	 * it explicitly.
	 */
	{
		struct bt_mesh_cfg_hb_sub sub = {
			.src = PUBLISHER_ADDR,
			.dst = GROUP_ADDR,
			.period = 0x10,
		};

		bt_mesh_cfg_hb_sub_set(net_idx, addr, &sub, NULL);
		printk("|   NRF58382   | Subscribing to heartbeat messages\n");
	}
#endif
}

static uint16_t target = GROUP_ADDR;

void board_button_1_pressed(void)
{
	NET_BUF_SIMPLE_DEFINE(msg, 3 + 4 + 4);
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = app_idx,
		.addr = target,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	/* Bind to Health model */
	bt_mesh_model_msg_init(&msg, OP_VENDOR_BUTTON);

	if (bt_mesh_model_send(&vnd_models[0], &ctx, &msg, NULL, NULL)) {
		printk("U|   NRF58382   | nable to send Vendor Button message\n");
	}

	printk("|   NRF58382   | Button message sent with OpCode 0x%08x\n", OP_VENDOR_BUTTON);
}
void mesh_send_start(uint16_t duration, int err, void *cb_data)
{
	printk("|   NRF58382   | send_start duration = %d, err = %d\n",duration,err);
}
void mesh_send_end(int err, void *cb_data)
{
	printk("|   NRF58382   | send_end err=%d\n\n\n",err);
};
const struct bt_mesh_send_cb kos_send_sb_s = { 
	.start = mesh_send_start,
	.end = mesh_send_end,
};
void sendDMBMessage(uint32_t data)
{
	int err;
	NET_BUF_SIMPLE_DEFINE(msg, 3 + 4 + 4);
	struct bt_mesh_msg_ctx ctx = {
		.app_idx = app_idx,
		.addr = target,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	bt_mesh_model_msg_init(&msg, OP_KOS_MESSAGE);	
	net_buf_simple_add_le32(&msg,data);
	err = bt_mesh_model_send(&vnd_models[0], &ctx, &msg,&kos_send_sb_s, NULL);
	if (err) {
		printk("|   NRF58382   | Unable to send KOS message %d\n",err);
	}

	printk("|   NRF58382   | KOS message sent with OpCode 0x%08x\n", OP_KOS_MESSAGE);
}
uint16_t board_set_target(void)
{
	switch (target) {
	case GROUP_ADDR:
		target = 1U;
		break;
	case 9:
		target = GROUP_ADDR;
		break;
	default:
		target++;
		break;
	}

	return target;
}


#define LOCK_PORT_BIT 2

void LockOff()
{
	gpio_pin_set(gpio0,LOCK_PORT_BIT,0);
}
void LockOn()
{
	gpio_pin_set(gpio0,LOCK_PORT_BIT,1);
}

void main(void)
{
	int err;	
	printk("|   NRF58382   | Initializing...\n");
	gpio0=device_get_binding("GPIO_0");
	if (gpio0 == NULL)
	{
		printk("|   NRF58382   | Error acquiring GPIO 0 interface\n");
		while(1);
	}
	err = gpio_pin_configure(gpio0,LOCK_PORT_BIT,GPIO_OUTPUT);
	if (err < 0)
	{
		printk("|   NRF58382   | Error configuring GPIO0 \n");
		while(1);
	}		

	printk("|   NRF58382   | Unicast address: 0x%04x\n", addr);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("|   NRF58382   | Bluetooth init failed (err %d)\n", err);
		return;
	}

	while(1)
	{		
		k_msleep(200);
	}	

}
