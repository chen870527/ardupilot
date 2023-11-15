//
// Simple test for the AP_AHRS interface
//

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();			//& 是取址運算符，它用於取得 AP_HAL::get_HAL() 返回對象的地址。整個語句的含義是創建一個對 AP_HAL::HAL 類型的常量引用 hal，並將其初始化為 AP_HAL::get_HAL() 返回的對象。
//?? AP_HAL::HAL 類型 有點confuse?


static AP_SerialManager serial_manager;	//AP_SerialManager 是 ArduPilot 中用來管理串口通信的類別。   ->命名為 serial_manager. 
AP_Int32 logger_bitmask;			//AP_Int32 是 ArduPilot 中定義的整數型別， ->命名為logger_bitmask.
static AP_Logger logger{logger_bitmask};	// 宣告了一個靜態變數，型別是 AP_Logger，名稱為 logger，並使用 logger_bitmask 作為初始化值。

class DummyVehicle : public AP_Vehicle {		// class DummyVehicle 繼承 AP_Vehicle  ，他是AP_Vehicle 類別的子類別
public:
    AP_AHRS ahrs{AP_AHRS::FLAG_ALWAYS_USE_EKF};		//這表示在 DummyVehicle 的每個實例中，都會有一個名為 ahrs 的 AP_AHRS 物件。 賦值為AP_AHRS::FLAG_ALWAYS_USE_EKF
    bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return true; };  //用來設置飛行模式。讓我分解一下 ->
    //bool: 這是函數的返回類型，表示這個函數返回一個布爾值（true 或 false）。
    //set_mode: 函數的名稱，用來設置飛行模式。
    //const uint8_t new_mode: 這是函數的第一個參數，類型是 uint8_t，表示一個無符號 8 位整數。這個參數名稱是 new_mode，它是用來接收新的飛行模式。
    //const ModeReason reason: 這是函數的第二個參數，類型是 ModeReason。這個參數名稱是 reason，它用來接收設置飛行模式的原因。
    //override: 這個關鍵字表示這個函數是一個虛函數，並且它覆蓋（重寫）了基類（基本類型或者說是父類）中的同名函數。
    //{ return true; }: 函數體，表示當這個函數被調用時，它會返回 true。
    
    uint8_t get_mode() const override { return 1; };//override 關鍵字是為了提醒編譯器，你打算覆蓋基類中的虛函數。它是一種檢查機制，有助於減少錯誤。 （override 可有可無 但最好是要有)
    //如果你聲明一個函數為虛函數（使用 virtual 關鍵字），但在派生類中沒有使用 override 關鍵字重新定義該函數，編譯器不會給出錯誤，但這樣的情況可能是一個錯誤，因為你原本可能打算覆蓋基類的函數。
    
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &task_count, uint32_t &log_bit) override {}; // ->用來獲取任務的調度器信息。
    // *&tasks，它用來接收任務的信息。  // &task_count 表示任務的數量。 // &log_bit表示日誌的位元。
    void init_ardupilot() override {}; 
    void load_parameters() override {};
    void init() {
        BoardConfig.init();// 調用 BoardConfig 的 init 函數
        ins.init(100); 	   // 調用 ins 的 init 函數，傳遞參數 100
        ahrs.init();	   // 調用 ahrs 的 init 函數
    }

};

static DummyVehicle vehicle;

// choose which AHRS system to use
// AP_AHRS_DCM ahrs = AP_AHRS_DCM::create(barometer, gps);
auto &ahrs = vehicle.ahrs;	//??? auto關鍵字，它的目的是自動推斷變數的類型。在這個情況下，&ahrs的類型將是AP_AHRS&，也就是vehicle.ahrs的引用。

void setup(void)	//初始化函數 setup
{
    vehicle.init();
    serial_manager.init();
    AP::compass().init(); //使用 AP::compass() 來獲取指南針（磁羅盤）的實例，然後再調用 init() 函數來進行初始化。AP 可能是一個命名空間（namespace）或者類別，而 compass() 可能是一個靜態成員函數
    if (!AP::compass().read()) {
        hal.console->printf("No compass detected\n");    // hal.console 是一個指向類型為 SomeClass 的對象的指針，那麼 ->printf("No compass detected\n"); 就是在調用 SomeClass 的 printf 函數。
    }
    AP::gps().init(serial_manager);
}

void loop(void)
{
    static uint16_t counter; //用於計算迴圈運行的次數。
    static uint32_t last_t, last_print, last_compass; // 記錄上一次迴圈運行的時間。
    uint32_t now = AP_HAL::micros(); // 表示當前的微秒時間。
    float heading = 0; 

    if (last_t == 0) {		//last_t 是否等於0。如果等於0，表示這是第一次進入 loop 函式，它會將 now 的值賦給 last_t，然後返回，結束這一次的函式loop的調用。
        last_t = now;		//這樣做的目的可能是在一開始的時候初始化一些變數或進行一些設定，而不是執行 loop 函式的主要邏輯。
        return;
    }
    last_t = now;

    if (now - last_compass > 100 * 1000UL &&	//UL 是一個後綴，表示這是一個無符號長整數（unsigned long）的字面值。在這個語境中，100 * 1000UL 表示100000，即100毫秒。
        AP::compass().read()) {			//這個條件語句檢查自上次讀取指南針以來是否已經過了100毫秒
        heading = AP::compass().calculate_heading(ahrs.get_rotation_body_to_ned());	//計算磁羅盤的方位角（heading）
        // read compass at 10Hz
        last_compass = now;
    }

    ahrs.update();	//更新姿態估計（AHRS）。
    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {
        Vector3f drift  = ahrs.get_gyro_drift();	//獲取陀螺儀的漂移（drift）。
        hal.console->printf(
                "r:%4.1f  p:%4.1f y:%4.1f " 
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n",
                (double)ToDeg(ahrs.roll),			//(double) -> 確保 ToDeg(ahrs.roll) 返回的值被解釋為雙精度浮點數
                (double)ToDeg(ahrs.pitch),
                (double)ToDeg(ahrs.yaw),
                (double)ToDeg(drift.x),
                (double)ToDeg(drift.y),
                (double)ToDeg(drift.z),
                (double)(AP::compass().use_for_yaw() ? ToDeg(heading) : 0.0f),
                (double)((1.0e6f * counter) / (now-last_print)));
        last_print = now;
        counter = 0;
    }
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
