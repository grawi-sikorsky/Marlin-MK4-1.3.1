/**
 * Nextion_lcd.cpp
 *
 * Copyright (c) 2014-2016 Alberto Cotronei @MagoKimbra
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl. If not, see <http://www.gnu.org/licenses/>.
 */
#include "../Marlin.h"
#include "../temperature.h"
#include "../cardreader.h"
//#include <MemoryFree.h>

#if ENABLED(NEXTION_DISPLAY)
	#include "../stepper.h"
	#include "../mesh_bed_leveling.h"
	#include "../configuration_store.h"
#endif

#if ENABLED(SPEAKER)
	#include "../buzzer.h"
#endif

#if ENABLED(NEXTION)

  #include "Nextion_lcd.h"
  #include "Nextion_gfx.h"
  #include "library/Nextion.h"

  bool        NextionON                 = false,
              show_Wave                 = true,
              lcdDrawUpdate             = false,
              lcd_clicked               = false;
  uint8_t     PageID                    = 0,
              lcd_status_message_level  = 0;
  uint16_t    slidermaxval              = 20;
  char        bufferson[70]             = { 0 };
  char        lcd_status_message[24]    = WELCOME_MSG;
  const float manual_feedrate_mm_m[]    = MANUAL_FEEDRATE;
	millis_t		screen_timeout_millis;

  extern uint8_t progress_printing; // dodane nex
	extern bool nex_filament_runout_sensor_flag;
	bool nex_m600_heatingup = 0;
	#if PIN_EXISTS(SD_DETECT)
	uint8_t lcd_sd_status;
	#endif

	extern float destination[XYZE];// = { 0.0 };
	extern bool g29_in_progress;// = false;
	extern inline void set_current_to_destination() { COPY(current_position, destination); }
	extern inline void set_destination_to_current() { COPY(destination, current_position); }
	extern void home_all_axes();

  #if ENABLED(SDSUPPORT)
    // 0 card not present, 1 SD not insert, 2 SD insert, 3 SD printing
    enum SDstatus_enum {NO_SD = 0, SD_NO_INSERT = 1, SD_INSERT = 2, SD_PRINTING = 3, SD_PAUSE = 4 };
    SDstatus_enum SDstatus    = NO_SD;
		#if ENABLED(NEX_UPLOAD)
			NexUpload Firmware(NEXTION_FIRMWARE_FILE, 57600);
		#endif
  #endif
	#if ENABLED(BABYSTEPPING)
			int _babystep_z_shift = 0;
	#endif


  #if ENABLED(NEXTION_GFX)
    GFX gfx = GFX(1, 1, 1, 1);
  #endif
	
	#if ENABLED(ADVANCED_PAUSE_FEATURE)
		void lcd_advanced_pause_wait_for_nozzles_to_heat();
		void lcd_advanced_pause_toocold_menu();
		void lcd_advanced_pause_option_menu();
		void lcd_advanced_pause_init_message();
		void lcd_advanced_pause_unload_message();
		void lcd_advanced_pause_insert_message();
		void lcd_advanced_pause_load_message();
		void lcd_advanced_pause_heat_nozzle();
		void lcd_advanced_pause_extrude_message();
		void lcd_advanced_pause_resume_message();
	#endif

  /**
   *******************************************************************
   * NEX lista stron uzytych
   *******************************************************************
   */

  //NexObject Pstart        = NexObject(0,  0,  "start");
  NexObject Pmenu         = NexObject(1,  0,  "menu");
  NexObject Pprinter      = NexObject(2,  0,  "printer");
  //NexObject Psdcard       = NexObject(3,  0,  "sdcard");
  NexObject Psetup        = NexObject(4,  0,  "setup");
	//NexObject Pmove         = NexObject(5,  0,  "move");// lipa
  //NexObject Pspeed        = NexObject(6,  0,  "speed");// lipa
  //NexObject Pgcode        = NexObject(7,  0,  "gcode");// lipa
  //NexObject Prfid         = NexObject(8,  0,  "rfid");
  //NexObject Pbrightness   = NexObject(9,  0,  "brightness");
  //NexObject Pinfo         = NexObject(10, 0,  "info");
  NexObject Pyesno        = NexObject(11, 0,  "yesno");
  NexObject Pfilament     = NexObject(12, 0, "filament");
  NexObject Pselect       = NexObject(13, 0,  "select");
  NexObject Pprobe        = NexObject(14, 0,  "bedlevel");
	NexObject Pheatup				= NexObject(15, 0,	"heatup");
	NexObject Poptions			= NexObject(16, 0,	"maintain");//
  //NexObject Ptime         = NexObject(17, 0,  "infomove");
  //NexObject Pfanspeedpage = NexObject(18, 0,  "fanspeedpage");
	//NexObject Pstats				= NexObject(19, 0,	"statscreen");
	//NexObject Ptsettings		= NexObject(20, 0,  "tempsettings");
	//NexObject Pinfobedlevel = NexObject(21, 0, "infobedlevel");
	//NexObject Pservice			= NexObject(22, 0, "servicepage");
	//NexObject Paccel				= NexObject(23, 0, "accelpage");
	//NexObject Pjerk					= NexObject(25, 0, "jerkpage");
	NexObject Pkill					= NexObject(30, 0, "killpage");
	// 
	// == 9 

  /**
   *******************************************************************
   * NEX komponenty strona: start
   *******************************************************************
   */
  NexObject startimer     = NexObject(0, 3, "tm1"); // out?
	// 
	// == 10

  /**
   *******************************************************************
   * NEX komponenty strona: menu
   *******************************************************************
   */
  NexObject Version     = NexObject(1, 6,  "t0"); // out? nieuzywane
	// 
	// == 11

  /**
   *******************************************************************
   * NEX komponenty strona: status/printer
   *******************************************************************
   */
  NexObject LcdX						= NexObject(2,  4,  "vx");
  NexObject LcdY						= NexObject(2,  5,  "vy");
  NexObject LcdZ						= NexObject(2,  6,  "vz");
  NexObject Hotend00				= NexObject(2,  7,  "he00");
  NexObject Hotend01				= NexObject(2,  8,  "he01");
  NexObject Bed0						= NexObject(2, 9,  "bed0");
	NexObject Bed1						= NexObject(2, 10, "bed1");
  NexObject SD							= NexObject(2, 11,  "sd");
  NexObject PrinterFanspeed = NexObject(2, 12,  "fs");
  NexObject VSpeed					= NexObject(2, 13,  "vs");
  NexObject Language				= NexObject(2, 14,  "lang");
  NexObject NStop						= NexObject(2, 18,  "p1");
  NexObject NPlay						= NexObject(2, 19,  "p2");
  NexObject LcdStatus				= NexObject(2, 20,  "t0");
  NexObject LcdTime					= NexObject(2, 21,  "t2");
  NexObject progressbar			= NexObject(2, 22,  "j0");
  NexObject Wavetemp				= NexObject(2, 23,  "s0");
	NexObject percentdone			= NexObject(2, 49,	"t4");
	//
	// == 18
	// == 29


  /**
   *******************************************************************
   * NEX komponenty strona: SDCard
   *******************************************************************
   */
  NexObject sdlist      = NexObject(3,   1, "h0");
  NexObject sdrow0      = NexObject(3,   2, "t0");
  NexObject sdrow1      = NexObject(3,   3, "t1");

  NexObject sdrow2      = NexObject(3,   4, "t2");
  NexObject sdrow3      = NexObject(3,   5, "t3");
  NexObject sdrow4      = NexObject(3,   6, "t4");
  NexObject sdrow5      = NexObject(3,   7, "t5");
  NexObject Folder0     = NexObject(3,  19, "p0");
  NexObject Folder1     = NexObject(3,  20, "p1");
  NexObject Folder2     = NexObject(3,  21, "p2");
  NexObject Folder3     = NexObject(3,  22, "p3");
  NexObject Folder4     = NexObject(3,  23, "p4");
  NexObject Folder5     = NexObject(3,  24, "p5");
  NexObject Folderup    = NexObject(3,  25, "p6");
  NexObject sdfolder    = NexObject(3,	 9, "t6");
  NexObject ScrollUp    = NexObject(3,  10, "p7");
  NexObject ScrollDown  = NexObject(3,  11, "p8");
#if ENABLED(NEXTION_SD_LONG_NAMES)
	NexObject file0				= NexObject(3, 12, "n1");
	NexObject file1				= NexObject(3, 13, "n2");
	NexObject file2				= NexObject(3, 14, "n3");
	NexObject file3				= NexObject(3, 15, "n4");
	NexObject file4				= NexObject(3, 16, "n5");
	NexObject file5				= NexObject(3, 17, "n6");
#endif
	// 
	// == 25
	// == 54


  /**
   *******************************************************************
   * NEX komponenty strona: MOVE
   *******************************************************************
   */
  NexObject XYHome      = NexObject(5,   2, "p4");
  NexObject XYUp        = NexObject(5,   3, "p5");
  NexObject XYRight     = NexObject(5,   4, "p6");
  NexObject XYDown      = NexObject(5,   5, "p7");
  NexObject XYLeft      = NexObject(5,   6, "p8");
  NexObject ZHome       = NexObject(5,   7, "p9");
  NexObject ZUp         = NexObject(5,   8, "p10");
  NexObject ZDown       = NexObject(5,   9, "p11");
  NexObject movecmd     = NexObject(5,  11, "vacmd");
  NexObject LedCoord5   = NexObject(5,  12, "t0");
  NexObject MotorOff    = NexObject(5,  17, "p0");
  NexObject Extrude     = NexObject(5,  19, "p12");	
  NexObject Retract     = NexObject(5,  20, "p14");
	// 
	// == 13
	// == 67

	/**
	*******************************************************************
	* NEX komponenty strona: SPEED
	*******************************************************************
	*/
	NexObject speedsetbtn	= NexObject(6, 9, "m0");
	NexObject SpeedNex		= NexObject(6, 7, "vspeed");
	// 
	// == 2
	// == 69

  /**
   *******************************************************************
   * NEX komponenty strona: GCode
   *******************************************************************
   */
  NexObject Tgcode      = NexObject(7,   1, "tgcode");
  NexObject Send        = NexObject(7,  25, "bsend");
	// 
	// == 2
	// == 71

  /**
   *******************************************************************
   * NEX komponenty strona: Info
   *******************************************************************
   */
  //NexObject InfoText    = NexObject(10, 2,  "t0");
  //NexObject ScrollText  = NexObject(10, 3,  "g0");
	// 
	//

  /**
   *******************************************************************
   * NEX komponenty strona: Yesno
   *******************************************************************
   */
  NexObject Vyes        = NexObject(11, 2,  "yn0");
  NexObject Riga0       = NexObject(11, 4,  "tl0");
  NexObject Riga1       = NexObject(11, 5,  "tl1");
  NexObject Riga2       = NexObject(11, 6,  "tl2");
  NexObject Riga3       = NexObject(11, 7,  "tl3");
  NexObject Yes         = NexObject(11, 8,  "p1");
  NexObject No          = NexObject(11, 9,  "p2");
	// 
	// == 7
	// == 78

  /**
   *******************************************************************
   * NEX komponenty strona:: Select
   *******************************************************************
   */
  NexObject LcdRiga1    = NexObject(13,  2, "t0");
  NexObject LcdRiga2    = NexObject(13,  3, "t1");
  NexObject LcdRiga3    = NexObject(13,  4, "t2");
  NexObject LcdRiga4    = NexObject(13,  5, "t3");
  NexObject LcdUp       = NexObject(13,  15, "p4");
  NexObject LcdSend     = NexObject(13,  14, "p1");
  NexObject LcdDown     = NexObject(13,  16, "p5");
  NexObject LcdMin      = NexObject(13,  7, "min");
  NexObject LcdMax      = NexObject(13, 8, "max");
  NexObject LcdPos      = NexObject(13, 9, "pos");
	// 
	// == 10
	// == 88

  /**
   *******************************************************************
   * NEX komponenty strona: Probe
   *******************************************************************
   */
  NexObject ProbeUp     = NexObject(14, 1,  "p0");
  NexObject ProbeSend   = NexObject(14, 2,  "p1");
  NexObject ProbeDown   = NexObject(14, 3,  "p2");
  //NexObject ProbeMsg    = NexObject(14, 4,  "t0");
  NexObject ProbeZ      = NexObject(14, 5,  "t1");
	// 
	// == 4
	// == 92


	/**
	*******************************************************************
	* NEX komponenty strona: HEATUP 15
	*******************************************************************
	*/
	NexObject heatupenter		= NexObject(15, 7, "m3");
	NexObject temphe				= NexObject(15, 8, "temphe");
	NexObject tempbe				= NexObject(15, 9, "tempbe");
	NexObject heatbedenter	= NexObject(15, 12, "m4");
	NexObject hotendenter		= NexObject(15, 13, "m5");
	NexObject chillenter		= NexObject(15, 14, "m6");
	// 
	// == 6
	// == 98

	/**
	*******************************************************************
	* NEX komponenty strona:: FAN SPEED 18
	*******************************************************************
	*/
	NexObject FanSpeedNex			= NexObject(18, 7, "vfan");
	NexObject FanSetBtn				= NexObject(18, 9, "m1");
	NexObject FanPageIDfrom		= NexObject(18, 10, "fanpagefrom");
	// 
	// == 3
	// == 101

	/**
	*******************************************************************
	* NEX komponenty strona:: STAT SCREEN 19
	*******************************************************************
	*/
	#if ENABLED(NEX_STAT_PAGE)
		NexObject statin			= NexObject(4, 1, "m2"); //przycisk z innej strony -> setup
		NexObject Sprints			= NexObject(19, 2, "t0");
		NexObject Scompl			= NexObject(19, 3, "t1");
		NexObject Spanic			= NexObject(19, 4, "t2");
		NexObject Stimetotal	= NexObject(19, 5, "t3");
		NexObject Stimelong		= NexObject(19, 6, "t4");
		NexObject Sfilament		= NexObject(19, 7, "t5");

		NexObject Sfirmware		= NexObject(19, 9, "t6");
		NexObject Skompil			= NexObject(19, 10, "t7");
		NexObject Sleveling		= NexObject(19, 11, "t8");
		NexObject Svlcs				= NexObject(19, 12, "t9");
		NexObject Sfilsensor	= NexObject(19, 13, "t10");
	#endif
	// 
	// == 12
	// == 113

	/**
	*******************************************************************
	* NEX komponenty strona:: SERVICE PAGE 22
	*******************************************************************
	*/
	//NexObject SvJerk				= NexObject(22, 4, "m2"); //wejscie w jerk -> przekazuje zmienne float na nuber nextion (brak dziesietnych)
	NexObject SvSteps				= NexObject(22, 5, "m3");	//wejscie w steps -> przekazuje zmienne float na nuber nextion (brak dziesietnych)
	// 
	// == 114

	/**
	*******************************************************************
	* NEX komponenty strona:: ACCEL SCREEN 23
	*******************************************************************
	*/
#if ENABLED(NEX_ACC_PAGE)
	NexObject accelin		= NexObject(4, 4, "m5");	// setaccelpagePopCallback -> przekazuje zmienne float do strony z akceleracja
	NexObject Awork			= NexObject(23, 22, "a0");
	NexObject Aretr			= NexObject(23, 23, "a1");
	NexObject Atravel		= NexObject(23, 24, "a2");
	NexObject Amaxx			= NexObject(23, 25, "a3");
	NexObject Amaxy			= NexObject(23, 26, "a4");
	NexObject Amaxz			= NexObject(23, 27, "a5");
	NexObject Amaxe			= NexObject(23, 28, "a6");
	NexObject Asend			= NexObject(23, 33, "p12"); 
	NexObject Asave			= NexObject(23, 30, "p10");	// setaccelsavebtnPopCallback -> wywo�uje settings.save();
	NexObject Aload			= NexObject(23, 31, "p11"); // setaccelloadbtnPopCallback	-> wywo�uje settings.load();
#endif
	// 
	// == 11
	// == 125

	/**
	*******************************************************************
	* NEX komponenty strona:: BABYSTEP SCREEN 28
	*******************************************************************
	*/
	NexObject ZbabyUp			= NexObject(28, 1, "m0");
	NexObject ZbabyDown		= NexObject(28, 2, "m1");
	NexObject ZbabyBack_Save = NexObject(28, 3, "m2");
	// 
	// == 3
	// == 128

	/**
	*******************************************************************
	* NEX komponenty strona: KILL SCREEN 30
	*******************************************************************
	*/
	NexObject Kmsg				= NexObject(30, 2, "tkmsg");
	// == 129

	/**
	*******************************************************************
	* NEX komponenty strona:: FLOWPAGE 31
	*******************************************************************
	*/
	NexObject vFlowNex					= NexObject(31, 7, "vflow");
	NexObject SetFlowBtn				= NexObject(31, 9, "m0");
	NexObject FlowPageFrom			= NexObject(31, 10, "flowfrom");
	// 
	// == 3
	// == 132
	// 132*13 = 1716 bajt�w

  NexObject *nex_listen_list[] =
  {
    // Page 2 touch listen
    &NPlay,

    // Page 3 touch listen
    &sdlist, &ScrollUp, &ScrollDown, &sdrow0, &sdrow1, &sdrow2,
    &sdrow3, &sdrow4, &sdrow5, &Folderup,// &sd_mount, &sd_dismount,

    // Page 4 touch listen setup
		#if ENABLED(NEX_STAT_PAGE)
				&statin, 
		#endif

		#if ENABLED(NEX_ACC_PAGE)
				&accelin,
				// Page 23 tacz listen
				&Asend, &Asave, &Aload,
		#endif

    // Page 5 touch listen
    &MotorOff, &XYHome, &XYUp, &XYRight, &XYDown, &XYLeft,
    &ZHome, &ZUp, &ZDown,
    &Extrude, &Retract,

		&speedsetbtn,

    // Page 7 touch listen
    &Send,

    // Page 11 touch listen
    &Yes, &No,

    // Page 12 touch listen
    //&FilLoad, &FilUnload, &FilExtr,

    // Page 13 touch listen
    &LcdSend,

    // Page 14 touch listen
    &ProbeUp, &ProbeDown, &ProbeSend,

		// Page 15 tacz listen
		&heatupenter, &heatbedenter, &hotendenter, &chillenter,

		// Page 18 tacz listen
		&FanSetBtn,

		//Page 22 service
		&SvSteps,

		// Page 28 babystep
		&ZbabyUp, &ZbabyDown, &ZbabyBack_Save,

		// Page 31 Flow
		&SetFlowBtn,

    NULL
  };

  NexObject *lcd_row_list[] =
  {
    &LcdRiga1,
    &LcdRiga2,
    &LcdRiga3,
    &LcdRiga4,
    NULL
  };

  NexObject *heater_list0[] =
  {
		&Hotend00,
		&Bed0,
    NULL
  };

  NexObject *heater_list1[] =
  {
		&Hotend01,
		&Bed1,
    NULL
  };

  NexObject *row_list[] =
  {
    &sdrow0,
    &sdrow1,
    &sdrow2,
    &sdrow3,
    &sdrow4,
    &sdrow5,
    NULL
  };

  NexObject *folder_list[] =
  {
    &Folder0,
    &Folder1,
    &Folder2,
    &Folder3,
    &Folder4,
    &Folder5,
    NULL
  };

	#if ENABLED(NEXTION_SD_LONG_NAMES)
		NexObject *file_list83[] =
		{
			&file0,
			&file1,
			&file2,
			&file3,
			&file4,
			&file5,
			NULL
		};
	#endif
	
  // Function pointer to menu functions.
  typedef void (*screenFunc_t)();

  /**
   *
   * Menu actions
   *
   */
  void lcd_sdcard_stop() {
		move_away_flag = false;									// flaga pause_print na false, na wypadek gdyby drukarka byla w stanie pauzy @_@
	  card.stopSDPrint();											// wstrzymaj wydruk z kartysd
		//clear_command_queue();									// czysc kolejke komend
		//stepper.quick_stop_panic();							// pomocne z panic'a, trzeba to zaserwowac aby mozna bylo ponownie wykonac jakakolwiek komende
		thermalManager.disable_all_heaters();		// wylacz grzalki
		percentdone.setText("0", "printer");		// zeruj procenty
		progressbar.setValue(0, "printer");			// zeruj progress bar

		#if ENABLED(PLOSS_SUPPORT)
			_babystep_z_shift = 0;								// dodane - zeruje babystep po zatrzymaniu wydruku
			eeprom_update_dword((uint32_t*)(EEPROM_PANIC_BABYSTEP_Z), _babystep_z_shift);	// zeruj babystepping w eeprom
		#endif
		#if FAN_COUNT > 0
			for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
		#endif

		wait_for_heatup = false;												// flaga false
		lcd_setstatusPGM(PSTR(MSG_PRINT_ABORTED), -1);	// status info
		print_job_timer.stop();			// wstrzymujemy timer

		//G28 on stop print
		#if ENABLED(STOP_PRINT_G28)
				//home_all_axes();						// bazujemy osie
		#endif
				enqueue_and_echo_commands_P(PSTR("G28"));
				//quickstop_stepper();
}

  //void menu_action_back() { Pprinter.show(); }
  void menu_action_function(screenFunc_t func) { (*func)(); }

	// Ustawia strone statusu przypisujac zerowe wartosci do zmiennych tj. glowica, stol, fan, stan SD
	// wywo�ywana raz w lcdinit()
  void setpagePrinter() 
	{
    char temp[8] = { 0 };

    #if HOTENDS > 0
      Hotend00.setValue(0, "printer");
			Hotend01.setValue(0, "printer");
    #endif

    #if HAS_TEMP_BED
      Bed0.setValue(0,"printer");
			Bed1.setValue(0,"printer");
    #endif

		#if FAN_COUNT > 0
			PrinterFanspeed.setValue(0, "printer");
		#endif

		VSpeed.setValue(100, "printer");

    #if ENABLED(SDSUPPORT)
      if (!card.cardOK) card.initsd();
      delay(100);
      if (card.cardOK) {
        SDstatus = SD_INSERT;
        card.setroot();  // Initial boot
      }
      else SDstatus = SD_NO_INSERT;

      SD.setValue(SDstatus, "printer");
    #endif

    #define LANGUAGE_STRING(M) STRINGIFY(M)
    #define NEXTION_LANGUAGE LANGUAGE_STRING(LCD_LANGUAGE)
    Language.setText(NEXTION_LANGUAGE, "printer");
  }

  void start_menu(const bool encoder=false, const bool push=false) 
	{
    Pselect.show();
    LcdUp.SetVisibility(encoder);
    LcdDown.SetVisibility(encoder);
    LcdSend.SetVisibility(push);
    lcdDrawUpdate = true;
    lcd_clicked = !push;
	}

	/**
	 * START_SCREEN  Opening code for a screen having only static items.s
	 *               Do simplified scrolling of the entire screen.
	 *
	 * START_MENU    Opening code for a screen with menu items.
	 *               Scroll as-needed to keep the selected line in view.
	 */

#define WAIT_FOR_CLICK_F(TYPE, ...) \
    if (lcd_clicked){ \
		menu_action_ ## TYPE(__VA_ARGS__); \
    return; }\

#define WAIT_FOR_CLICK() \
    if (lcd_clicked){ \
			nex_m600_heatingup = 0;\
			Pfilament.show();\
    return; }\

  #define START_SCREEN() \
    start_menu(false, true); \
    do { \
      uint8_t _lcdLineNr = 0; \

  #define START_MENU() \
    start_menu(true, true); \
    uint16_t encoderLine = 1; \
    uint8_t _lcdLineNr = 0; \
    do { \
      _lcdLineNr = 0; \
      encoderLine = LcdPos.getValue(); \
      delay(100)

  #define MENU_ITEM(TYPE, LABEL, ...) \
      if (lcdDrawUpdate) { \
        lcd_row_list[_lcdLineNr]->setText_PGM(PSTR(LABEL)); \
        LcdMax.setValue(_lcdLineNr); \
      } \
      if (lcd_clicked && encoderLine == _lcdLineNr) { \
        menu_action_ ## TYPE(__VA_ARGS__); \
        return; \
      } \
      ++_lcdLineNr

  #define MENU_BACK(LABEL) MENU_ITEM(back, LABEL)

  #define STATIC_ITEM_P(LABEL) \
      if (lcdDrawUpdate) { \
        lcd_row_list[_lcdLineNr]->setText_PGM(LABEL); \
        LcdMin.setValue(_lcdLineNr + 1); \
      } \
      ++_lcdLineNr \

  #define STATIC_ITEM(LABEL) STATIC_ITEM_P(PSTR(LABEL))

  #define END_MENU() \
      idle(); \
      lcdDrawUpdate = false; \
    } while(1)

  #define END_SCREEN() \
      lcdDrawUpdate = false; \
    } while(0)

	 // Portions from STATIC_ITEM...
	#define HOTEND_STATUS_ITEM() do { \
        if (lcdDrawUpdate) { \
          lcd_row_list[_lcdLineNr]->setText(itostr3(thermalManager.degHotend)); \
        } \
				lcdDrawUpdate = true; \
				++_lcdLineNr; \
    }while(0)



// ========================
// SDSUPPORT
// ========================
  #if ENABLED(SDSUPPORT)

	#if ENABLED(NEX_UPLOAD)
	void UploadNewFirmware() {
		if (IS_SD_INSERTED || card.cardOK) {
			Firmware.startUpload();
			nexSerial.end();
			lcd_init();
		}
	}
	#endif

	#if ENABLED(NEXTION_SD_LONG_NAMES)
		//Drukuje linijke na stronie SDCARD
		//1 iterator
		//2 folder
		//3 nazwa 8.3
		//4 nazwa do wyswietlenia
		void printrowsd(uint8_t row, const bool folder, const char* filename, const char* longfilename) {
				if (folder) {
					folder_list[row]->SetVisibility(true);
					row_list[row]->attachPop(sdfolderPopCallback, row_list[row]);
				} else if (filename == "") {
					folder_list[row]->SetVisibility(false);
					row_list[row]->detachPop();
				} else {
					folder_list[row]->SetVisibility(false);
					row_list[row]->attachPop(sdfilePopCallback, row_list[row]);
				}
				file_list83[row]->setText(filename);
				row_list[row]->setText(longfilename);
			}
	#else
		//1 iterator
		//2 folder
		//3 nazwa 8.3
	void printrowsd(uint8_t row, const bool folder, const char* filename) {
		if (folder) {
			folder_list[row]->SetVisibility(true);
			row_list[row]->attachPop(sdfolderPopCallback, row_list[row]);
		}
		else if (filename == "") {
			folder_list[row]->SetVisibility(false);
			row_list[row]->detachPop();
		}
		else {
			folder_list[row]->SetVisibility(false);
			row_list[row]->attachPop(sdfilePopCallback, row_list[row]);
		}
		row_list[row]->setText(filename);
	}
	#endif

		//Ustawia liste plikow na stronie SDCARD
    static void setrowsdcard(uint32_t number = 0) {
      uint16_t fileCnt = card.getnrfilenames(); // nalezaloby przeniesc funkcje get_num_files z mk4duo / jakis tweak
      uint32_t i = 0;
      card.getWorkDirName();

      if (card.filename[0] != '/') {
        Folderup.SetVisibility(true);
        Folderup.attachPop(sdfolderUpPopCallback);
        sdfolder.setText(card.filename);
      } else {
        Folderup.detachPop();
        Folderup.SetVisibility(false);
        sdfolder.setText("");
      }

      if (fileCnt >= 0) {
        for (uint8_t row = 0; row < 6; row++) {
          i = row + number;
          if (i < fileCnt) {
            #if ENABLED(SDCARD_SORT_ALPHA)
              card.getfilename_sorted(i);
            #else
              card.getfilename(i);
            #endif

						#if ENABLED(NEXTION_SD_LONG_NAMES)
							printrowsd(row, card.filenameIsDir, card.filename, card.longFilename);
						#else
							printrowsd(row, card.filenameIsDir, card.filename);
						#endif

          } else {
						#if ENABLED(NEXTION_SD_LONG_NAMES)
							printrowsd(row, false, "", "");
						#else
							printrowsd(row, false, "");
						#endif
						
          }
        }
      }
      //sendCommand("ref 0");
    }

		//
		// Funkcja z obsluga klikniecia w linijke z nazwa pliku
		// Zapisuje do EEPROM sciezke pliku oraz babystep (VLCS)
    static void menu_action_sdfile(const char* filename) 
		{
			#if ENABLED(PLOSS_SUPPORT) // jezeli VLCS wlaczony
				for (int i = 0; i < 8; i++) {
					eeprom_write_byte((uint8_t*)EEPROM_SD_FILENAME + i, filename[i]);
				}

				uint8_t depth = (uint8_t)card.getWorkDirDepth();
				eeprom_write_byte((uint8_t*)EEPROM_SD_FILE_DIR_DEPTH, depth);

				for (uint8_t i = 0; i < depth; i++) {
					for (int j = 0; j < 8; j++) {
						eeprom_write_byte((uint8_t*)EEPROM_SD_DIRS + j + 8 * i, dir_names[i][j]);
					}
				}
				_babystep_z_shift = 0;																												// zeruj babystep po uruchomieniu wydruku
				eeprom_update_dword((uint32_t*)(EEPROM_PANIC_BABYSTEP_Z), _babystep_z_shift);	// zeruj babystepping w eeprom
			#endif // jezeli VLCS wlaczone

      card.openAndPrintFile(filename);
      Pprinter.show();
    }

		//
		// Funkcja z obsluga klikniecia w linijke z nazwa FOLDERU
		//
    static void menu_action_sddirectory(const char* filename) {
			#if ENABLED(PLOSS_SUPPORT)
				uint8_t depth = (uint8_t)card.getWorkDirDepth();	// dodane	
				strcpy(dir_names[depth], filename);								// dodane
			#endif	//OCB?

      card.chdir(filename);
      setpageSD();
    }

		// Ustawia strone z karta SD
		// obsluga slidera
    void setpageSD() {
      uint16_t fileCnt = card.getnrfilenames();

      if (fileCnt <= 6)
        slidermaxval = 0;
      else
        slidermaxval  = fileCnt - 6;

			uint16_t hig = 210 - slidermaxval * 10;
      if (hig < 10) hig = 10;
		
      sdlist.Set_cursor_height_hig(hig);	

      sdlist.setMaxval(slidermaxval);
      sdlist.setValue(slidermaxval,"sdcard");
      //sendCommand("ref 0"); ref 0 jest w setrowsdcard()

      setrowsdcard();
    }

		// Obsluga slidera / suwaka
    void sdlistPopCallback(void *ptr) {
      UNUSED(ptr);
      uint16_t number = slidermaxval - sdlist.getValue();
      setrowsdcard(number);
    }

		// NEXTION: Obsluga klikniecia linijek tekstu z nazwa pliku do druku
    void sdfilePopCallback(void *ptr) {
      ZERO(bufferson);
			#if ENABLED(NEXTION_SD_LONG_NAMES)
				if (ptr == &sdrow0)
					file0.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow1)
					file1.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow2)
					file2.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow3)
					file3.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow4)
					file4.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow5)
					file5.getText(bufferson, sizeof(bufferson));
			#else
				if (ptr == &sdrow0)
					sdrow0.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow1)
					sdrow1.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow2)
					sdrow2.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow3)
					sdrow3.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow4)
					sdrow4.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow5)
					sdrow5.getText(bufferson, sizeof(bufferson));
			#endif

			buzzer.tone(100, 2300);
      menu_action_sdfile(bufferson);
    }

		// NEXTION: Obsluga klikniecia linijek tekstu z nazwa FOLDERU
    void sdfolderPopCallback(void *ptr) {
      ZERO(bufferson);
			#if ENABLED(NEXTION_SD_LONG_NAMES)
				if (ptr == &sdrow0)
					file0.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow1)
					file1.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow2)
					file2.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow3)
					file3.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow4)
					file4.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow5)
					file5.getText(bufferson, sizeof(bufferson));
			#else
				if (ptr == &sdrow0)
					sdrow0.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow1)
					sdrow1.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow2)
					sdrow2.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow3)
					sdrow3.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow4)
					sdrow4.getText(bufferson, sizeof(bufferson));
				else if (ptr == &sdrow5)
					sdrow5.getText(bufferson, sizeof(bufferson));
			#endif
      menu_action_sddirectory(bufferson);
			buzzer.tone(100, 2300);
    }

		// NEXTION: Obsluga klikniecia przycisku Folder Up
    void sdfolderUpPopCallback(void *ptr) {
      UNUSED(ptr);
      card.updir();
      setpageSD();
			buzzer.tone(100, 2300);
    }

		// NEXTION: Obsluga klikniecia linijek przycisku PAUSE / PLAY
    void PlayPausePopCallback(void *ptr) {
      UNUSED(ptr);
      if (card.cardOK && card.isFileOpen()) {
        if (IS_SD_PRINTING) {														//pause
          card.pauseSDPrint();
          print_job_timer.pause();
          #if ENABLED(PARK_HEAD_ON_PAUSE)
            enqueue_and_echo_commands_P(PSTR("M125"));
          #endif
					lcd_setstatusPGM(PSTR(MSG_PRINT_PAUSED));
        }
        else {																					//resume
					#if ENABLED(PARK_HEAD_ON_PAUSE)
						enqueue_and_echo_commands_P(PSTR("M24"));
					#else
						card.startFileprint();
						print_job_timer.start();
					#endif
					lcd_setstatusPGM(PSTR(MSG_RESUME_PRINT));
        }
				buzzer.tone(100, 2300);
      }
    }
  #endif 
// ========================
// END OF SDSUPPORT
// ========================


// ========================
// FILAMENT CHANGE M600
// ========================
  #if ENABLED(ADVANCED_PAUSE_FEATURE)

    static AdvancedPauseMenuResponse advanced_pause_mode = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;

		void lcd_advanced_pause_toocold_menu() {
			nex_m600_heatingup = 1; // wlacz wyswietlanie temperatury
			//screen_timeout_millis = millis(); // wlaczamy timer
			START_SCREEN();
			STATIC_ITEM(MSG_TOO_COLD_FOR_M600_1);
			STATIC_ITEM(MSG_TOO_COLD_FOR_M600_2);
			STATIC_ITEM(MSG_TOO_COLD_FOR_M600_3);
			STATIC_ITEM(MSG_TOO_COLD_FOR_M600_4);
			WAIT_FOR_CLICK();
			END_MENU();
		}

		void nex_enqueue_filament_change() {
			#if ENABLED(PREVENT_COLD_EXTRUSION)
				if (!DEBUGGING(DRYRUN) && !thermalManager.allow_cold_extrude &&
					thermalManager.degTargetHotend(active_extruder) < thermalManager.extrude_min_temp) {
					buzzer.tone(120, 700); // dodane beeper too cold
					buzzer.tone(120, 000);
					buzzer.tone(120, 700);
					lcd_advanced_pause_toocold_menu();
					return;
				}
			#endif
			Pselect.show();
			enqueue_and_echo_commands_P(PSTR("M600 B0"));
		}

    static void lcd_advanced_pause_resume_print() {
      advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
      Pprinter.show();
    }

    static void lcd_advanced_pause_extrude_more() {
      advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
    }

    static void lcd_advanced_pause_option_menu() {
      START_MENU();
      STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_OPTION_HEADER);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_RESUME, lcd_advanced_pause_resume_print);
      MENU_ITEM(function, MSG_FILAMENT_CHANGE_OPTION_EXTRUDE, lcd_advanced_pause_extrude_more);
      END_MENU();
    }

    static void lcd_advanced_pause_init_message() {
      START_SCREEN();
			STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INIT_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_unload_message() {
      START_SCREEN();
			STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER);
			STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_1);
			STATIC_ITEM(MSG_FILAMENT_CHANGE_UNLOAD_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_wait_for_nozzles_to_heat() {
      START_SCREEN();
			STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEATING_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_heat_nozzle() {
      START_SCREEN();
			STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEAT_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_HEAT_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_insert_message() {
      START_SCREEN();
			STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_2);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_INSERT_3);
      END_SCREEN();
    }

    static void lcd_advanced_pause_load_message() {
      START_SCREEN();
			STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_LOAD_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_purge_message() {
      START_SCREEN();
			//STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER); usuniete bo przy pauzie rowniez bylo wyswietlane
      STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_EXTRUDE_2);
      END_SCREEN();
    }

    static void lcd_advanced_pause_resume_message() {
      START_SCREEN();
			//STATIC_ITEM(MSG_NEX_FILAMENT_CHANGE_HEADER); jw.
      STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_1);
      STATIC_ITEM(MSG_FILAMENT_CHANGE_RESUME_2);
      END_SCREEN();
    }

    void lcd_advanced_pause_show_message(const AdvancedPauseMessage message,const AdvancedPauseMenuResponse mode/*=ADVANCED_PAUSE_MODE_PAUSE_PRINT*/) 
		{
      //UNUSED(extruder);
      static AdvancedPauseMessage old_message;
      advanced_pause_mode = mode;

      if (old_message != message) {
				nex_m600_heatingup = 0;//zmiana jesli wyjdzie poza heatingup 
        switch (message) {
          case ADVANCED_PAUSE_MESSAGE_INIT:
            lcd_advanced_pause_init_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_UNLOAD:
            lcd_advanced_pause_unload_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_INSERT:
            lcd_advanced_pause_insert_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_LOAD:
            lcd_advanced_pause_load_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_EXTRUDE:
            lcd_advanced_pause_purge_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_RESUME:
            lcd_advanced_pause_resume_message();
            break;
          case ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE:
            lcd_advanced_pause_heat_nozzle();
            break;
          case ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT:
						nex_m600_heatingup = 1;
            lcd_advanced_pause_wait_for_nozzles_to_heat();
            break;
          case ADVANCED_PAUSE_MESSAGE_OPTION:
            advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
            lcd_advanced_pause_option_menu();
            break;
          //case ADVANCED_PAUSE_MESSAGE_STATUS:
          default:
            Pprinter.show();
            break;
        }
        old_message = message;
      }
    }

  #endif // ADVANCED_PAUSE_FEATURE
// =============================
// END OF FILAMENT CHANGE M600
// =============================


// ======================
// VLCS LCD SUPPORT
// ======================
#ifdef PLOSS_SUPPORT
		// ODPOWIEDZI
		void lcd_ploss_menu_response_yes() {
			lcd_ploss_menu_response = PLOSS_LCD_RESPONSE_YES;
		}
		void lcd_ploss_menu_response_no() {
			lcd_ploss_menu_response = PLOSS_LCD_RESPONSE_NO;
		}

		//MENU
		void ploss_recovery_menu() {
			START_MENU();
			STATIC_ITEM("Wykryto zanik napiecia");
			STATIC_ITEM("Czy wznowic wydruk?");
			MENU_ITEM(function, "Tak", lcd_ploss_menu_response_yes);
			MENU_ITEM(function, "Nie", lcd_ploss_menu_response_no);
			END_MENU();
		}

		void ploss_recovery_menu_resuming() {
			Pselect.show();
			screen_timeout_millis = millis(); //wlaczamy screen timeout
			START_SCREEN();
			LcdSend.SetVisibility(false);
			STATIC_ITEM("Wznawianie wydruku");
			STATIC_ITEM("po zaniku zasilania");
			END_SCREEN();
	}

		void ploss_recovery_menu_last_confirm() {
			Pselect.show();
			screen_timeout_millis = 0; // wylaczamy screen timeout
			START_SCREEN();
			STATIC_ITEM("Usun nadmiar");
			STATIC_ITEM("filamentu i kliknij");
			STATIC_ITEM("aby wznowic wydruk");
			WAIT_FOR_CLICK_F(function, lcd_ploss_menu_response_yes); // return response if clicked
			END_MENU();
		}

		void lcd_ploss_menu_last_info() {
			START_SCREEN();
			STATIC_ITEM("Wznawianie wydruku");
			STATIC_ITEM("");
			END_SCREEN();
			screen_timeout_millis = millis();
		}

		void ploss_recovery_menu_no_resume() {
			screen_timeout_millis = millis();
			START_SCREEN();
			STATIC_ITEM("Uruchamianie");
			STATIC_ITEM("po zaniku zasilania");
			STATIC_ITEM("Bazowanie osi");
			END_SCREEN();
		}

		void lcd_ploss_recovery_menu(const PlossMenuMessage message) {
			switch (message) {
			case PLOSS_LCD_MANUAL_RECOVERY:
				lcd_ploss_menu_response = PLOSS_LCD_RESPONSE_WAIT_FOR_USER;
				ploss_recovery_menu();
				break;
			case PLOSS_LCD_RECOVERY_RESUMING:
				ploss_recovery_menu_resuming();
				break;
			case PLOSS_LCD_MENU_NO_RESUME:
				ploss_recovery_menu_no_resume();
				enqueue_and_echo_commands_P(PSTR("G28"));
				break;
			case PLOSS_LCD_MENU_LAST_CONFIRM:
				lcd_ploss_menu_response = PLOSS_LCD_RESPONSE_WAIT_FOR_LAST_CONFIRMATION;
				ploss_recovery_menu_last_confirm();
				break;
			case PLOSS_LAST_INFO:
				lcd_ploss_menu_last_info();
				break;
			case PLOSS_LCD_AUTO_RECOVERY:
				break;
			}
		}
#endif //PLOSS
// ======================
// VLCS LCD SUPPORT   ===
// ======================


  #if ENABLED(RFID_MODULE)
    void rfidPopCallback(void *ptr) {
      ZERO(bufferson);

      String temp = "M522 ";
      uint16_t Rfid_read = RfidR.getValue();

      if (ptr == &Rfid0)
        temp += "T0 ";
      else if (ptr == &Rfid1)
        temp += "T1 ";
      else if (ptr == &Rfid2)
        temp += "T2 ";
      else if (ptr == &Rfid3)
        temp += "T3 ";
      else if (ptr == &Rfid4)
        temp += "T4 ";
      else if (ptr == &Rfid5)
        temp += "T5 ";

      if(Rfid_read)
        temp += "R";
      else
        temp += "W";

      temp.toCharArray(bufferson, sizeof(bufferson));
      commands.enqueue_and_echo(bufferson);
    }

    void rfid_setText(const char* message, uint32_t color /* = 65535 */) {
      char Rfid_status_message[25];
      strncpy(Rfid_status_message, message, 30);
      RfidText.Set_font_color_pco(color);
      RfidText.setText(Rfid_status_message);
    }
  #endif

// =======================
// BED LEVELING SUPPORT ==
// =======================
#if ENABLED(NEXTION_BED_LEVEL)
    void ProbelPopCallBack(void *ptr) {

      if (ptr == &ProbeUp || ptr == &ProbeDown) {

				set_destination_to_current();

        if (ptr == &ProbeUp)
          destination[Z_AXIS] += (LCD_Z_STEP);
        else
          destination[Z_AXIS] -= (LCD_Z_STEP);

        NOLESS(destination[Z_AXIS], -(LCD_PROBE_Z_RANGE) * 0.5);
        NOMORE(destination[Z_AXIS], (LCD_PROBE_Z_RANGE) * 0.5);

        const float old_feedrate = feedrate_mm_s;
        feedrate_mm_s = MMM_TO_MMS(manual_feedrate_mm_m[Z_AXIS]);
        prepare_move_to_destination(); // will call set_current_from_destination()
        feedrate_mm_s = old_feedrate;

        stepper.synchronize();
      }
      else if (ptr == &ProbeSend) {
				SERIAL_ECHOLNPGM("probesend:");
        #if HAS_LEVELING && ENABLED(NEXTION_BED_LEVEL)
				if (g29_in_progress == true) {
					enqueue_and_echo_commands_P(PSTR("G29 S2")); 
				}
        #endif
					wait_for_user = false;
      }
    }

		void nex_return_after_leveling(bool finish)
		{
			if (finish == true)
			{
				Pprinter.show();
			}
		}
#endif
// ==============================
// END OF BED LEVELING SUPPORT ==
// ==============================


  void sethotPopCallback(void *ptr) {
    UNUSED(ptr);

		uint16_t	temp_hotend = temphe.getValue(), //dodane
							temp_bed = tempbe.getValue();    //dodane

		thermalManager.setTargetHotend(temp_hotend, 0);
		thermalManager.setTargetBed(temp_bed);

    Pprinter.show();
		buzzer.tone(100,2300);
  }

	void sethotendPopCallback(void *ptr) {
		UNUSED(ptr);
		uint16_t	temp_hotend = temphe.getValue();
		thermalManager.setTargetHotend(temp_hotend, 0);
		Pprinter.show();
		buzzer.tone(100, 2300);
	}

	void setheatbedPopCallback(void *ptr) {
		UNUSED(ptr);
		uint16_t temp_bed = tempbe.getValue();    //dodane
		thermalManager.setTargetBed(temp_bed);
		Pprinter.show();
		buzzer.tone(100, 2300);
	}

	void setfanandgoPopCallback(void *ptr) {
		uint8_t fanpagefrom, vfanbuff;
		UNUSED(ptr);
		ZERO(bufferson);
		vfanbuff = FanSpeedNex.getValue("fanspeedpage");
		fanpagefrom = FanPageIDfrom.getValue("fanspeedpage");
		fanSpeeds[0] = vfanbuff;
		if (fanpagefrom == 0) // wejscie z status
		{
			Pprinter.show();
		}
		else if (fanpagefrom == 1) // wejscie z heatup
		{
			Pheatup.show();
		}
	}
	#if ENABLED(NEX_STAT_PAGE)
		void setsetupstatPopCallback(void *ptr)
		{
			UNUSED(ptr);
				// PRINTSTATS START
				char buffer[21];
				printStatistics stats = print_job_timer.getStats();

				Sprints.setText(itostr3left(stats.totalPrints),"statscreen");        // Print Count: 999
				Scompl.setText(itostr3left(stats.finishedPrints), "statscreen");			// Completed  : 666

				#if ENABLED(PLOSS_SUPPORT)
					Spanic.setText(itostr3left(eeprom_read_byte((uint8_t*)EEPROM_PANIC_POWER_FAIL_COUNT)), "statscreen"); // dodane power fail count
				#endif

				duration_t elapsed = stats.printTime;
				elapsed.toString(buffer);
				Stimetotal.setText(buffer, "statscreen");               // Total print Time: 99y 364d 23h 59m 59s

				elapsed = stats.longestPrint;
				elapsed.toString(buffer);
				Stimelong.setText(buffer, "statscreen");								// Longest job time: 99y 364d 23h 59m 59s

				sprintf_P(buffer, PSTR("%ld.%im"), long(stats.filamentUsed / 1000), int16_t(stats.filamentUsed / 100) % 10);
				Sfilament.setText(buffer, "statscreen");								// Extruded total: 125m
				// END OF PRINTSTATS

				// PRINTER INFO START
				Sfirmware.setText_PGM(PSTR(SHORT_BUILD_VERSION), "statscreen");
				Skompil.setText_PGM(PSTR(STRING_DISTRIBUTION_DATE), "statscreen");
				Sleveling.setText_PGM(PSTR(MSG_MESH_LEVELING), "statscreen");
				#if ENABLED(PLOSS_SUPPORT)
							Svlcs.setText_PGM(PSTR(MSG_INFO_YES), "statscreen");
				#else
							Svlcs.setText_PGM(PSTR(MSG_INFO_NO), "statscreen");
				#endif

				#if ENABLED(FILAMENT_RUNOUT_SENSOR)
							if (eeprom_read_byte((uint8_t*)EEPROM_NEX_FILAMENT_SENSOR) == 1)
							{
								Sfilsensor.setText_PGM(PSTR(MSG_INFO_YES), "statscreen");
							}
							else
							{
								Sfilsensor.setText_PGM(PSTR(MSG_INFO_NO), "statscreen");
							}
				#else
							Sfilsensor.setText_PGM(PSTR(MSG_INFO_NO), "statscreen");
				#endif			
				// END OF PRINTER INFO

		}
	#endif
#if ENABLED(NEX_ACC_PAGE)
	void setaccelpagePopCallback(void *ptr)
	{
			UNUSED(ptr);
			Awork.setValue(planner.acceleration, "accelpage"); //va0
			Aretr.setValue(planner.retract_acceleration, "accelpage");	//va1
			Atravel.setValue(planner.travel_acceleration, "accelpage");
			Amaxx.setValue(planner.max_acceleration_mm_per_s2[X_AXIS], "accelpage");
			Amaxy.setValue(planner.max_acceleration_mm_per_s2[Y_AXIS], "accelpage");
			Amaxz.setValue(planner.max_acceleration_mm_per_s2[Z_AXIS], "accelpage");
			Amaxe.setValue(planner.max_acceleration_mm_per_s2[E_AXIS+active_extruder], "accelpage");
	}
	void getaccelPagePopCallback(void *ptr)
	{
		planner.acceleration = Awork.getValue("accelpage");
		planner.retract_acceleration = Aretr.getValue("accelpage");
		planner.travel_acceleration = Atravel.getValue("accelpage");
		planner.max_acceleration_mm_per_s2[X_AXIS] = Amaxx.getValue("accelpage");
		planner.max_acceleration_mm_per_s2[Y_AXIS] = Amaxy.getValue("accelpage");
		planner.max_acceleration_mm_per_s2[Z_AXIS] = Amaxz.getValue("accelpage");
		planner.max_acceleration_mm_per_s2[E_AXIS + active_extruder] = Amaxe.getValue("accelpage");
	}
	void setjerkpagePopCallback(void *ptr)
	{
			UNUSED(ptr);
			Awork.setValue(planner.max_jerk[X_AXIS], "accelpage"); //va0
			Aretr.setValue(planner.max_jerk[Y_AXIS], "accelpage");	//va1
			Atravel.setValue(planner.max_jerk[Z_AXIS],"accelpage"); //va2
			Amaxx.setValue(planner.max_jerk[E_AXIS],"accelpage");	//va3
	}

	void setstepspagePopCallback(void *ptr)
	{
			UNUSED(ptr);
			Awork.setValue(planner.axis_steps_per_mm[X_AXIS], "accelpage"); //va0
			Aretr.setValue(planner.axis_steps_per_mm[Y_AXIS], "accelpage");	//va1
			Atravel.setValue(planner.axis_steps_per_mm[Z_AXIS], "accelpage");	//va2
			Amaxx.setValue(planner.axis_steps_per_mm[E_AXIS+active_extruder], "accelpage");	//va3
	}
#endif
	void setaccelsavebtnPopCallback(void *ptr)
	{
		settings.save();
		SERIAL_ECHOPGM("zapisane");
	}
	void setaccelloadbtnPopCallback(void *ptr)
	{
		settings.load();
		SERIAL_ECHOPGM("zaladowane");
	}

	void setBabystepUpPopCallback(void *ptr)
	{
		nextion_babystep_z(false);
	}
	void setBabystepDownPopCallback(void *ptr)
	{
		nextion_babystep_z(true);
	}
	void setBabystepEEPROMPopCallback(void *ptr)
	{
		eeprom_update_dword((uint32_t*)(EEPROM_PANIC_BABYSTEP_Z), _babystep_z_shift);
	}
	void setspeedPopCallback(void *ptr) 
	{
		int vspeedbuff;
		UNUSED(ptr);
		vspeedbuff = (int)SpeedNex.getValue("speed");

		feedrate_percentage = vspeedbuff;
		Pprinter.show();
	}
	void setflowPopCallback(void *ptr)
	{
		uint8_t flowfrom;
		int vflowbuff;
		UNUSED(ptr);
		vflowbuff = (int)vFlowNex.getValue("flowpage");
		flowfrom = FlowPageFrom.getValue("flowpage");

		flow_percentage[0] = vflowbuff;

		if (flowfrom == 0) // wejscie z status
		{
			Pprinter.show();
		}
		else if (flowfrom == 1) // wejscie z heatup
		{
			Poptions.show();
		}
	}

  void setgcodePopCallback(void *ptr) {
    UNUSED(ptr);
    ZERO(bufferson);
		
    Tgcode.getText(bufferson, sizeof(bufferson), "gcode");
    Tgcode.setText("", "gcode");

		if (strcmp(bufferson,"M600") == 0)
		{
			nex_enqueue_filament_change();
		}
		else if (strcmp(bufferson, "M78 S78") == 0)
		{
			enqueue_and_echo_command(bufferson);
			buzzer.tone(100, 2300);
		}
		else
		{ 
			enqueue_and_echo_command(bufferson);
		}
  }

  void setmovePopCallback(void *ptr) {
    UNUSED(ptr);
    ZERO(bufferson);
    movecmd.getText(bufferson, sizeof(bufferson));
		enqueue_and_echo_commands_P(PSTR("G91"));
		enqueue_and_echo_command(bufferson);
		enqueue_and_echo_commands_P(PSTR("G90"));
  }

  void motoroffPopCallback(void *ptr) {
    UNUSED(ptr);
		enqueue_and_echo_commands_P(PSTR("M84"));
  }

	// NEXTION: Obsluga klikniecia przycisku SEND na SELECT PAGE
  void sendPopCallback(void *ptr) {
    UNUSED(ptr);
    lcd_clicked = true;
		wait_for_user = false;

		// dodane aby wyjsc kliknieciem z ostatniego ekranu vlcs / oraz z ekranu too cold for extrude m600
		if (screen_timeout_millis != 0)
		{
			screen_timeout_millis = 0;
			Pprinter.show();
		}
  }

	// NEXTION: Obsluga klikniecia YES oraz NO
  void YesNoPopCallback(void *ptr) {
    if (ptr == &Yes) {
      switch(Vyes.getValue()) {
        #if ENABLED(SDSUPPORT)
          case 1: // Stop Print
						Pprinter.show();
						lcd_sdcard_stop();
            break;
          case 2: // Upload Firmware
						#if ENABLED(NEX_UPLOAD)
            UploadNewFirmware(); 
						#endif
						break;
        #endif
        #if HAS_SD_RESTART
          case 3: // Restart file
            Pprinter.show();
            restart.start_job();
            break;
        #endif
        case 4: // Unconditional stop
          Pprinter.show();
          break;
				case 5: // ustaw czujnik filamentu
					nex_filament_runout_sensor_flag = 1;
					eeprom_update_byte((uint8_t*)EEPROM_NEX_FILAMENT_SENSOR,1);
					Psetup.show();
					break;
        default: break;
      }
    }
    else {
      switch(Vyes.getValue()) {
        #if ENABLED(SDSUPPORT)
          case 2:
            Psetup.show(); break;
        #endif
        #if HAS_SD_RESTART
          case 3:
            card.printingHasFinished();
            Pprinter.show();
            break;
        #endif
					case 5: // ustaw czujnik filamentu
						nex_filament_runout_sensor_flag = 0;
						eeprom_update_byte((uint8_t*)EEPROM_NEX_FILAMENT_SENSOR, 0);
						Psetup.show();
						break;
        default:
          Pprinter.show(); break;
      }
    }
  }

// =======================
// ==	LCD INIT					==
// =======================
  void lcd_init() {
    for (uint8_t i = 0; i < 10; i++) {
      ZERO(bufferson);
      NextionON = nexInit(bufferson);
      if (NextionON) break;
      delay(20);
    }

		#if ENABLED(FSENSOR_STATE)
			nex_filament_runout_sensor_flag = eeprom_read_byte((uint8_t*)EEPROM_NEX_FILAMENT_SENSOR);
		#endif

		#if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
			SET_INPUT_PULLUP(SD_DETECT_PIN);
			lcd_sd_status = 2; // UNKNOWN
		#endif

    if (!NextionON) {
	  SERIAL_ECHOPGM("Nextion not connected!");
      return;
    }
    else {
		SERIAL_ECHO_START();
	  SERIAL_ECHOPGM("Nextion");

    if (strstr(bufferson, "3224")) {       // Model 2.4" or 2.8" Normal or Enhanced
			SERIAL_ECHOPGM(" 2.4");
        #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
        #endif
      }
    else if (strstr(bufferson, "4024")) {  // Model 3.2" Normal or Enhanced
			SERIAL_ECHOPGM(" 3.2");
       #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
       #endif
      }
		else if (strstr(bufferson, "4832")) {  // Model 3.2" Normal or Enhanced
			SERIAL_ECHOPGM(" 3.5");
			#if ENABLED(NEXTION_GFX)
				gfx.set_position(1, 24, 250, 155);
			#endif
			}
    else if (strstr(bufferson, "4827")) {  // Model 4.3" Normal or Enhanced
			SERIAL_ECHOPGM(" 4.3");
      #if ENABLED(NEXTION_GFX)
          gfx.set_position(1, 24, 250, 155);
      #endif
      }
    else if (strstr(bufferson, "8048")) {  // Model 7" Normal or Enhanced
			SERIAL_ECHOPGM(" 7");
      #if ENABLED(NEXTION_GFX)
          gfx.set_position(274, 213, 250, 155);
      #endif
      }
	  SERIAL_CHAR('"'); SERIAL_ECHOLNPGM(" connected!");

      #if ENABLED(NEXTION_GFX)
        gfx.color_set(NX_AXIS + X_AXIS, 63488);
        gfx.color_set(NX_AXIS + Y_AXIS, 2016);
        gfx.color_set(NX_AXIS + Z_AXIS, 31);
        gfx.color_set(NX_MOVE, 2047);
        gfx.color_set(NX_TOOL, 65535);
        gfx.color_set(NX_LOW, 2047);
        gfx.color_set(NX_HIGH, 63488);
      #endif

			//
			// NEX USTAWIENIE PRZYCISKOW
			//
			// SDSUPPORT
      #if ENABLED(SDSUPPORT)
        sdlist.attachPop(sdlistPopCallback);
        ScrollUp.attachPop(sdlistPopCallback);
        ScrollDown.attachPop(sdlistPopCallback);
        NPlay.attachPop(PlayPausePopCallback);
      #endif

			// BED LEVEL
      #if ENABLED(NEXTION_BED_LEVEL)
        ProbeUp.attachPush(ProbelPopCallBack, &ProbeUp);
        ProbeSend.attachPop(ProbelPopCallBack, &ProbeSend);
        ProbeDown.attachPush(ProbelPopCallBack, &ProbeDown);
      #endif

			// STATS
			#if ENABLED(NEX_STAT_PAGE)
				statin.attachPop(setsetupstatPopCallback); //dodane info o wejsciu w statystyki
			#endif

			// ACCELERATION
			#if ENABLED(NEX_ACC_PAGE)
				accelin.attachPop(setaccelpagePopCallback); //setaccelpagePopCallback
				Asend.attachPop(getaccelPagePopCallback);
				Asave.attachPop(setaccelsavebtnPopCallback);
				Aload.attachPop(setaccelloadbtnPopCallback);
			#endif

			SvSteps.attachPop(setstepspagePopCallback);

			// TEMPERATURA
			heatupenter.attachPop(sethotPopCallback, &heatupenter); // obsluga przycisku rozgrzej oba
			hotendenter.attachPop(sethotendPopCallback, &hotendenter); //obsluga przycisku rozgrzej hotend
			heatbedenter.attachPop(setheatbedPopCallback, &heatbedenter); //obsluga przycisku rozgrzej bed
			chillenter.attachPop(sethotPopCallback, &chillenter); //obs�uga przycisku chlodzenie

			FanSetBtn.attachPop(setfanandgoPopCallback); //obsluga przycisku fan set

			speedsetbtn.attachPop(setspeedPopCallback); //obsluga przycisku speed set

			SetFlowBtn.attachPop(setflowPopCallback); //obsluga przycisku set flow

			// BABYSTEP
			ZbabyUp.attachPush(setBabystepUpPopCallback);	// obsluga przycisku babystep up
			ZbabyDown.attachPush(setBabystepDownPopCallback); // obsluga przycisku babystep down
			ZbabyBack_Save.attachPop(setBabystepEEPROMPopCallback);
			
			// MOVE PAGE
      XYHome.attachPop(setmovePopCallback);
			XYUp.attachPush(setmovePopCallback);
      XYRight.attachPush(setmovePopCallback);
      XYDown.attachPush(setmovePopCallback);
      XYLeft.attachPush(setmovePopCallback);
      ZHome.attachPop(setmovePopCallback);
      ZUp.attachPush(setmovePopCallback);
      ZDown.attachPush(setmovePopCallback);
      Extrude.attachPush(setmovePopCallback);
      Retract.attachPush(setmovePopCallback);
      MotorOff.attachPop(motoroffPopCallback);

			// GCODE
      Send.attachPop(setgcodePopCallback);

			// YESNO
      Yes.attachPop(YesNoPopCallback, &Yes);
      No.attachPop(YesNoPopCallback, &No);

			// SELECT PAGE
      LcdSend.attachPop(sendPopCallback);

      setpagePrinter();
      startimer.enable();

			buzzer.tone(100, 2300); // dodane - wejsciowy brzeczyk
			buzzer.tone(100, 2600);
			buzzer.tone(100, 3100);

			Pmenu.show();
    }
  }
// =======================
// == END OF	LCD INIT	==
// =======================

  bool lcd_detected() { return NextionON; }

	// Wysyla aktualne temperatury do NEX
  static void degtoLCD(const uint8_t h, float temp) {
    NOMORE(temp, 350);//999

    heater_list0[h]->setValue(temp,"printer");

    #if ENABLED(NEXTION_GFX)
      if (!(print_job_counter.isRunning() || IS_SD_PRINTING) && !Wavetemp.getObjVis() && show_Wave) {
        Wavetemp.SetVisibility(true);
      }
    #endif
  }

	// Wysyla docelowe temperatury do NEX
  static void targetdegtoLCD(const uint8_t h, const float temp) {
    heater_list1[h]->setValue(temp);
  }

	// Wysyla koordynaty do do NEX / MOVE PAGE / STATUS / BEDLEVEL
  static void coordtoLCD() {
    char* valuetemp;
		static float temppos[2];
    ZERO(bufferson);
    if (PageID == 2) {
			// if sprawdza czy nastapila zmiana pozycji aby nie spamowalo po serialu pozycja bez zmian -> todo: przeniesc na 8 bit...
			if( current_position[X_AXIS] != temppos[X_AXIS] )
			{
				LcdX.setText(ftostr41sign(LOGICAL_X_POSITION(current_position[X_AXIS])),"printer");
				temppos[X_AXIS] = current_position[X_AXIS];
			}
			if( current_position[Y_AXIS] != temppos[Y_AXIS] )
			{
				LcdY.setText(ftostr41sign(LOGICAL_Y_POSITION(current_position[Y_AXIS])),"printer");
				temppos[Y_AXIS] = current_position[Y_AXIS];
			}
			if( current_position[Z_AXIS] != temppos[Z_AXIS] )
			{
				LcdZ.setText(ftostr41sign(FIXFLOAT(LOGICAL_Z_POSITION(current_position[Z_AXIS]))),"printer");
				temppos[Z_AXIS] = current_position[Z_AXIS];
			}
    }
    else if (PageID == 5) {
      if (axis_homed[X_AXIS]) {
        valuetemp = ftostr4sign(LOGICAL_X_POSITION(current_position[X_AXIS]));
        strcat(bufferson, "X");
        strcat(bufferson, valuetemp);
      }
      else
        strcat(bufferson, "?");

      if (axis_homed[Y_AXIS]) {
        valuetemp = ftostr4sign(LOGICAL_Y_POSITION(current_position[Y_AXIS]));
        strcat(bufferson, " Y");
        strcat(bufferson, valuetemp);
      }
      else
        strcat(bufferson, " ?");

      if (axis_homed[Z_AXIS]) {
        valuetemp = ftostr52sp(FIXFLOAT(LOGICAL_Z_POSITION(current_position[Z_AXIS])));
        strcat(bufferson, " Z");
        strcat(bufferson, valuetemp);
      }
      else
        strcat(bufferson, " ?");

      LedCoord5.setText(bufferson,"move");
    }
    else if (PageID == 14) {
      ProbeZ.setText(ftostr43sign(FIXFLOAT(LOGICAL_Z_POSITION(current_position[Z_AXIS]))),"bedlevel");
    }
  }

	// Sprawdza obecnosc karty SD i montuje/odmontowuje karte na ekranie
	// IS_SD_INSERTED ma odwrocona logike:
	// 1 - brak karty
	// 0 - karta wlozona
	void nex_check_sdcard_present()
	{
	#if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
		// IS_SD_INSERTED ma odwrocona logike:
		// 1 - brak karty
		// 0 - karta wlozona
		const bool sd_status = IS_SD_INSERTED;
		if (sd_status != lcd_sd_status && lcd_detected())								// sprawdz czy nastapila zmiana? SD DET ->
		{																																// TAK:
			SERIAL_ECHOLNPGM("zmiana sd det:");
			if (!sd_status)																									// je�li SD_DETECT == false:
			{
				SERIAL_ECHOLNPGM("sd_status:false");
				card.initsd();																								// inicjalizacja karty
				setpageSD();																									// ustaw strone i przekaz flage do strony status
				SDstatus = SD_INSERT;
				SD.setValue(SDstatus, "printer");
				if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_INSERTED);			// MSG
			}
			else																														// je�li SD_DETECT == true:
			{
				SERIAL_ECHOLNPGM("sd_status:true");
				card.release();																								// odmontuj kart� SD
				setpageSD();																									// ustaw strone i przekaz flage do strony status
				SDstatus = SD_NO_INSERT;
				SD.setValue(SDstatus, "printer");
				if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_REMOVED);				// MSG
			}
			lcd_sd_status = sd_status;
		} // CALY IF SPRAWDZA STAN SD_DETECT I JEGO ZMIANE: SD jest->init / SD niet->release
	#endif
	}

// =======================
// == LCD UPDATE				==
// =======================
  void lcd_update() {
    if (!NextionON) return;
    nexLoop(nex_listen_list); // odswieza sie z delayem 5 ms

		//sprawdzamy timeout ekranu
		millis_t timeout_check;
		timeout_check = millis();
		if (timeout_check > screen_timeout_millis + NEX_SCREEN_TIME && screen_timeout_millis != 0)
		{
			Pprinter.show();
			screen_timeout_millis = 0;
		}
  }

// ===========================
// == LCD PERIODICAL UPDATE	==
// ===========================
  void nextion_draw_update() {

    static uint8_t  PreviousPage = 0,
                    Previousfeedrate = 0,
                    PreviousfanSpeed = 0,
										Previousflow = 0,
                    PreviouspercentDone = 0;
    static float    PreviousdegHeater[1] = { 0.0 },
                    PrevioustargetdegHeater[1] = { 0.0 };

    if (!NextionON) return;
	
    PageID = Nextion_PageID();
		if(PageID == 100 || PageID == 101)	PageID = PreviousPage;		// jesli na serialu lipa (przyczyna?) to loop do nastepnej proby

    switch(PageID)
		{
      case 2:
        if (PreviousPage != 2) 
				{
					lcd_setstatus(lcd_status_message);
          #if ENABLED(NEXTION_GFX)
            #if MECH(DELTA)
              gfx_clear(mechanics.delta_print_radius * 2, mechanics.delta_print_radius * 2, mechanics.delta_height);
            #else
              gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
            #endif
          #endif
				}
				//fanek
         if (PreviousfanSpeed != fanSpeeds[0]) {
					PrinterFanspeed.setValue(((float)(fanSpeeds[0]) / 255) * 100,"printer");
          PreviousfanSpeed = fanSpeeds[0];
         }
				//feedrate
        if (Previousfeedrate != feedrate_percentage) {
          VSpeed.setValue(feedrate_percentage,"printer");
          Previousfeedrate = feedrate_percentage;
        }
				//flow
				if (Previousflow != flow_percentage[0]) {
					vFlowNex.setValue(flow_percentage[0], "flowpage");
					Previousflow = flow_percentage[0];
				}
        #if HAS_TEMP_0
          if (PreviousdegHeater[0] != thermalManager.current_temperature[0]) 
					{
						PreviousdegHeater[0] = thermalManager.current_temperature[0];
            degtoLCD(0, PreviousdegHeater[0]);
          }
          if (PrevioustargetdegHeater[0] != thermalManager.target_temperature[0]) 
					{
			  PrevioustargetdegHeater[0] = thermalManager.target_temperature[0];
            targetdegtoLCD(0, PrevioustargetdegHeater[0]);
          }
        #endif
				#if HAS_TEMP_BED
					if (PreviousdegHeater[1] != thermalManager.current_temperature_bed) {
						PreviousdegHeater[1] = thermalManager.current_temperature_bed;
						degtoLCD(1, PreviousdegHeater[1]);
					}
					if (PrevioustargetdegHeater[1] != thermalManager.target_temperature_bed) {
						PrevioustargetdegHeater[1] = thermalManager.target_temperature_bed;
						targetdegtoLCD(1, PrevioustargetdegHeater[1]);
					}
				#endif
 
        coordtoLCD();
				
				if (PreviouspercentDone != progress_printing) {
					// Progress bar solid part
					progressbar.setValue(progress_printing,"printer");
					// Estimate End Time
					ZERO(bufferson);
					char buffer1[10];
					uint8_t digit;
					duration_t Time = print_job_timer.duration();
					digit = Time.toDigital(buffer1, true);
					strcat(bufferson, "S");
					strcat(bufferson, buffer1);
					Time = (print_job_timer.duration() * (100 - progress_printing)) / (progress_printing + 0.1);
					digit += Time.toDigital(buffer1, true);
					if (digit > 14)
						strcat(bufferson, "E");
					else
						strcat(bufferson, " E");
					strcat(bufferson, buffer1);
					LcdTime.setText(bufferson,"printer");
					PreviouspercentDone = progress_printing;

					// procenty t4
					ZERO(bufferson);
					strcat(bufferson, itostr3(progress_printing));
					strcat(bufferson, " %");
					percentdone.setText(bufferson, "printer");
				}
				else
				{
					ZERO(bufferson);
					strcat(bufferson, itostr3(progress_printing));
					strcat(bufferson, " %");
					percentdone.setText(bufferson, "printer");
					progressbar.setValue(progress_printing, "printer"); // dodatkowo odswiez progressbar
				}

				#if ENABLED(SDSUPPORT)
				if (card.isFileOpen()) {
					if (IS_SD_PRINTING && SDstatus != SD_PRINTING) {
						SDstatus = SD_PRINTING;
						SD.setValue(SDstatus,"printer");
					}
					else if (!IS_SD_PRINTING && SDstatus != SD_PAUSE) {
						SDstatus = SD_PAUSE;
						SD.setValue(SDstatus,"printer");
					}
				}
				else if (card.cardOK && SDstatus != SD_INSERT) {
					SDstatus = SD_INSERT;
					SD.setValue(SDstatus,"printer");
				}
				else if (card.cardOK && SDstatus != SD_NO_INSERT) {
					SDstatus = SD_NO_INSERT;
					SD.setValue(SDstatus,"printer");
				}
				#endif // HAS_SD_SUPPORT

				#if HAS_SD_RESTART
          if (restart.count && restart.job_phase == RESTART_IDLE) {
            restart.job_phase = RESTART_MAYBE; // Waiting for a response
            lcd_yesno(3, MSG_RESTART_PRINT, "", MSG_USERWAIT);
          }
        #endif 
        break;
	#if ENABLED(SDSUPPORT)
      case 3:
					if (PreviousPage != 3) {
						setpageSD();
					}
					nex_check_sdcard_present(); // sprawdz obecnosc karty sd, mount/unmount // potencjalnie tutaj jest bug z odswiezajacym sie ekranem SD 
          break;
	#endif
      case 5:
        coordtoLCD();
        break;
      case 6:
        //Previousfeedrate = feedrate_percentage = (int)VSpeed.getValue("printer");
        break;
			case 12:
				// odswiez temp glowicy na ekranie filament [przyciski]
					degtoLCD(0, thermalManager.current_temperature[0]);
				break;
			case 13:
				// pokaz temp glowicy podczas nagrzewania m600 na stronie select
				if (nex_m600_heatingup == 1)
				{
					char *temp_he;
					char *temp_te;
					char temptemp[14];

					temp_te = itostr3(thermalManager.target_temperature[0]);
					temp_he = itostr3(thermalManager.current_temperature[0]);
					strlcpy(temptemp,temp_he,4);
					strcat_P(temptemp, PSTR(" / "));
					strcat(temptemp, itostr3(thermalManager.target_temperature[0]));
					LcdRiga4.setText(temptemp);
				}
				break;
      case 14:
        coordtoLCD();
        break;
			case 31:
				vFlowNex.setValue(flow_percentage[0], "flowpage");
				break;
    }
    PreviousPage = PageID;
  }
	
  void lcd_setstatus(const char* message, bool persist) {
    UNUSED(persist);
    if (lcd_status_message_level > 0 || !NextionON) return;
    strncpy(lcd_status_message, message, 24);
    if (PageID == 2) LcdStatus.setText(lcd_status_message);
  }

  void lcd_setstatusPGM(const char* message, int8_t level) {
    if (level < 0) level = lcd_status_message_level = 0;
    if (level < lcd_status_message_level || !NextionON) return;
    strncpy_P(lcd_status_message, message, 24);
    lcd_status_message_level = level;
    if (PageID == 2) LcdStatus.setText(lcd_status_message);
  }

  void lcd_status_printf_P(const uint8_t level, const char * const fmt, ...) {
    //if (level < lcd_status_message_level || !NextionON) return;
    //lcd_status_message_level = level;
    //va_list args;
    //va_start(args, fmt);
    //vsnprintf(lcd_status_message, 24, fmt, args);
    //va_end(args);
    //if (PageID == 2) LcdStatus.setText(lcd_status_message);
  }

  void lcd_setalertstatusPGM(const char * const message) {
    lcd_setstatusPGM(message, 1);
  }

  void lcd_reset_alert_level() { lcd_status_message_level = 0; }


  void lcd_yesno(const uint8_t val, const char* msg1, const char* msg2, const char* msg3) {
    Vyes.setValue(val, "yesno");
    Pyesno.show();
    Riga0.setText(msg1);
    Riga1.setText(msg2);
    Riga3.setText(msg3);
  }

	void lcd_nextion_kill_msg(const char* lcd_msg)
	{
		Pkill.show();
		Kmsg.setText_PGM(lcd_msg,"killpage");
	}

	// dodana obsluga babystep
	#if ENABLED(BABYSTEPPING)
		void nextion_babystep_z(bool dir) {
				const int16_t babystep_increment = 8;

				if (dir == true)
				{
					thermalManager.babystep_axis(Z_AXIS, babystep_increment);
					_babystep_z_shift += babystep_increment;
				}
				else if (dir == false)
				{
					thermalManager.babystep_axis(Z_AXIS, -babystep_increment);
					_babystep_z_shift -= babystep_increment;
				}
		}
	#endif


  #if ENABLED(NEXTION_GFX)
    void gfx_origin(const float x, const float y, const float z) {
      gfx.origin(x, y, z);
    }

    void gfx_scale(const float scale) {
      gfx.set_scale(scale);
    }

    void gfx_clear(const float x, const float y, const float z, bool force_clear) {
      if (PageID == 2 && (print_job_counter.isRunning() || IS_SD_PRINTING || force_clear)) {
        Wavetemp.SetVisibility(false);
        show_Wave = !force_clear;
        gfx.clear(x, y, z);
      }
    }

    void gfx_cursor_to(const float x, const float y, const float z, bool force_cursor) {
      if (PageID == 2 && (print_job_counter.isRunning() || IS_SD_PRINTING || force_cursor))
        gfx.cursor_to(x, y, z);
    }

    void gfx_line_to(const float x, const float y, const float z) {
      if (PageID == 2 && (print_job_counter.isRunning() || IS_SD_PRINTING)) {
        #if ENABLED(ARDUINO_ARCH_SAM)
          gfx.line_to(NX_TOOL, x, y, z, true);
        #else
          gfx.line_to(NX_TOOL, x, y, z);
        #endif
      }
    }

    void gfx_plane_to(const float x, const float y, const float z) {
      uint8_t color;
      if (PageID == 2) {
        if (z < 10) color = NX_LOW;
        else color = NX_HIGH;
        gfx.line_to(color, x, y, z, true);
      }
    }
  #endif

#endif
