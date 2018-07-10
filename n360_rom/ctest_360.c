#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <stdint.h>
#include <libdragon.h>


//Prototypes
void graphics_draw_custom(display_context_t disp, int x, int y, char* symbol);

char *format_type( int accessory )
{
	switch( accessory )
	{
		case ACCESSORY_RUMBLEPAK:
			return "(rumble)";
		case ACCESSORY_MEMPAK:
			return "(memory)";
		case ACCESSORY_VRU:
			return "(vru)   ";
		default:
			return "(no pak)";
	}
}

int main(void)
{
	init_interrupts();
	display_init(RESOLUTION_320x240, DEPTH_32_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE );
	dfs_init(DFS_DEFAULT_LOCATION);
	controller_init();

	int press = 0;
	uint32_t frame_count=0;
	
	uint8_t data[32]; memset(data, 0, 32);
	
	static char up_arrow[5] = {
		0b00000100,
		0b00001110,
		0b00011011,
		0b00010001,
		0b00000000
	};
	static char down_arrow[5] = {
		0b00010001,
		0b00011011,
		0b00001110,
		0b00000100,
		0b00000000
	};
	static char left_arrow[5] = {
		0b00110000,
		0b01100000,
		0b11000000,
		0b01100000,
		0b00110000
	};
	static char right_arrow[5] = {
		0b00001100,
		0b00000110,
		0b00000011,
		0b00000110,
		0b00001100
	};
	static char marker[5] = {
		0b00000000,
		0b00011000,
		0b00111100,
		0b00011000,
		0b00000000
	};
	
	
	char render_buffer[80];
	
	/* Main loop test */
	while(1) 
	{
		static display_context_t disp = 0;
		while(!(disp = display_lock()));	
		graphics_fill_screen(disp, 0);

		/* To do initialize routines */
		controller_scan();
		struct controller_data keys = get_keys_down();
		int controllers = get_controllers_present();
		static uint16_t address=0x0000;
		
		sprintf(render_buffer,"A/B:   start/stop  rumble\n");
		graphics_draw_text(disp, 20, 2*8, render_buffer);
		sprintf(render_buffer,"Z/L+Z: read/write mempak at 0x%.4x\n",address);
		graphics_draw_text(disp, 20, 3*8, render_buffer);	
		sprintf(render_buffer,"Data: ");
		graphics_draw_text(disp, 20, 4*8, render_buffer);
		
		if(frame_count>5){
			for(int i = 0; i < 4; i++){
				if(keys.c[i].A){
					rumble_start(i);
				}

				if( keys.c[i].B){
					rumble_stop(i);
				}
				
				if(keys.c[i].Z){
					press = read_mempak_address(i, address, data);
				}
		
				if(keys.c[i].L && keys.c[i].Z){
					press = write_mempak_address(i, address, data);
				}			
				
				if(keys.c[i].up){
					address+=32;
					if(address>0x7FFF) address=0x0000;	
				}
				else if(keys.c[i].down){
					address-=32;
					if(address>0x9FFF) address=0x7FFF;
				}
				
				if(keys.c[i].C_up){
					address+=32*8;
					if(address>0x7FFF) address=0x0000;	
				}
				else if(keys.c[i].C_down){
					address-=32*8;
					if(address>0x9FFF) address=0x7FFF;
				}			
			}		
		}
			
		//Print data read from mempack.
		graphics_set_color(graphics_make_color(66,161,244,255),0x00000000); //Light Blue
		for(int i = 0; i < 32; i++) {
			sprintf(render_buffer,"%.2x\n", data[i]);	
			if(i<16){
				graphics_draw_text(disp, 20+(i*8*2), 5*8, render_buffer);
			} else {
				graphics_draw_text(disp, 20+((i-16)*8*2), 6*8, render_buffer);
			}
		}
	
		graphics_set_color(graphics_make_color(255,255,255,255),0x00000000); //Return to White
		
		
		if(press==0){
				graphics_set_color(graphics_make_color(0,255,0,255),0x00000000); //Green
				graphics_draw_text(disp, 40, 7*8, "CRC ok\n");	
		} else {
				graphics_set_color(graphics_make_color(255,0,0,255),0x00000000); //Red
				graphics_draw_text(disp, 40, 7*8, "CRC error\n");	
		}
		
		graphics_set_color(graphics_make_color(255,255,255,255),0x00000000); //Return to White

		graphics_draw_custom(disp,210,67,up_arrow);
		graphics_draw_custom(disp,218,67,down_arrow);
		graphics_draw_custom(disp,229,66,left_arrow);
		graphics_draw_custom(disp,234,66,right_arrow);
		
		graphics_draw_custom(disp,242,67,up_arrow);
		graphics_draw_custom(disp,250,67,down_arrow);
		graphics_draw_custom(disp,261,66,left_arrow);
		graphics_draw_custom(disp,266,66,right_arrow);
		
		graphics_draw_text(disp, 180, 9*8,  "ABZSCCCCDDDDLR");
		
		struct controller_data keys_now = get_keys_pressed();
		for (int i=0;i<4;i++){
			
			//Check if controller(s) present. Prints green if present. Red otherwise.
			if(controllers & 0xF000>>(i*4)){
				graphics_set_color(graphics_make_color(0,255,0,255),0x00000000); //Green
			} else {
				graphics_set_color(graphics_make_color(255,0,0,255),0x00000000); //Red
			}
			//Prints what peripheral is installed
			sprintf(render_buffer,"Cont %.1x\n",i+1);
			graphics_draw_text(disp, 20,        10*8+(i*8),render_buffer);
			graphics_draw_text(disp, 20+(7*8), 10*8+(i*8),format_type(identify_accessory(i)));
			graphics_set_color(graphics_make_color(255,255,255,255),0x00000000); //Return to White
			
			//Print key presses
			struct SI_condat* k = keys_now.c;
			sprintf(render_buffer,"%.1x%.1x%.1x%.1x",k[i].A,k[i].B,k[i].Z,k[i].start);
			sprintf(render_buffer+4,"%.1x%.1x%.1x%.1x",k[i].C_up,k[i].C_down,k[i].C_left,k[i].C_right);
			sprintf(render_buffer+8,"%.1x%.1x%.1x%.1x",k[i].up,k[i].down,k[i].left,k[i].right);
			sprintf(render_buffer+12,"%.1x%.1x"		,k[i].L,k[i].R);
			graphics_draw_text(disp, 180, (10+i)*8,  render_buffer);
			
			sprintf(render_buffer,"x:%4i\n",k[i].x);
			graphics_draw_text(disp, 20, (16+i)*8,  render_buffer);
			sprintf(render_buffer,"y:%4i\n",k[i].y);
			graphics_draw_text(disp, 72, (16+i)*8,  render_buffer);
			
			//Drawing analog stick map
			int x_pos=k[i].x;
			int y_pos=k[i].y;
			if(x_pos>100)x_pos=100;
			if(x_pos<-100)x_pos=-100;
			if(y_pos>100)y_pos=100;
			if(y_pos<-100)y_pos=-100;
			const int x_datum = 229;
			const int y_datum = 165;
			const int col = graphics_make_color(100,100,100,255);
	 
			graphics_draw_line(disp,x_datum,y_datum-50,x_datum,y_datum+50,col); //Y axis
			graphics_draw_line(disp,x_datum-50,y_datum,x_datum+50,y_datum,col); //X axis
			
			graphics_draw_line(disp,x_datum+41,y_datum,x_datum+41-9,y_datum-34,col); //Quadrant 1
			graphics_draw_line(disp,x_datum+41-9,y_datum-34,x_datum,y_datum-41,col); //Quadrant 1
			
			graphics_draw_line(disp,x_datum-41,y_datum,x_datum-41+9,y_datum-34,col); //Quadrant 2
			graphics_draw_line(disp,x_datum-41+9,y_datum-34,x_datum,y_datum-41,col); //Quadrant 2
			
			graphics_draw_line(disp,x_datum-41,y_datum,x_datum-41+9,y_datum+34,col); //Quadrant 3
			graphics_draw_line(disp,x_datum-41+9,y_datum+34,x_datum,y_datum+41,col); //Quadrant 3
			
			graphics_draw_line(disp,x_datum+41,y_datum,x_datum+41-9,y_datum+34,col); //Quadrant 4
			graphics_draw_line(disp,x_datum+41-9,y_datum+34,x_datum,y_datum+41,col); //Quadrant 4

			graphics_draw_custom(disp,x_datum+x_pos/2-3,y_datum-y_pos/2-2,marker);
			
			
		}
		
		frame_count++;
		display_show(disp);
		
	}
}


void graphics_draw_custom(display_context_t disp, int x, int y, char* symbol){
	static uint32_t f_color = 0xFFFFFFFF;
	static uint32_t b_color = 0x00000000;
	for(int row = 0; row < 5; row++ )
        {
            unsigned char c = symbol[row];
            for( int col = 0; col < 8; col++ )
            {
                /* Display foreground or background depending on font data */
                graphics_draw_pixel(disp, x + col, y + row, (c & 0x80) ? f_color : b_color );
                c <<= 1;
            }
     }
}


