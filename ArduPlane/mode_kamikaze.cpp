/*
Teknofest 2023 Savasan IHA Kamikaze görevi için geliştirilmiştir.

Özellikler
 - dalış kontrolcüsü
 - dalış başlangıç irtifası
 - dalış açısı
 - minimum irtifa
 - hızlı kalkış manevrası

Yeni parametreler

 - KAMI_START_ALT   : Dalış başlangıç irtifası
 - KAMI_DIVE_ANGLE  : Dalış Açısı
 - KAMI_END_ALT     : Dalışın sonlanacağı irtifa
 - KAMI_RISE_ANGLE  : Kalkışta uygulanacak pitch açısı (kullanilmiyor)
 - KAMI_CNTRL_P     : Dalış kontrolcüsü P katsayısı
 - KAMI_CNTRL_I     : Dalış kontrolcüsü I katsayısı 
 - KAMI_RAD_OFFSET  : Dalış yarıçapı offseti
 - KAMI_LAT_BAS     : Dalış konumu ilk enlem parametresi
 - KAMI_LAT_SON     : Dalış konumu ikinci enlem parametresi
 - KAMI_LON_BAS     : Dalış konumu ilk boylam parametresi
 - KAMI_LON_SON     : Dalış konumu ikinci boylam parametresi
 - KAMI_QR_DETECT   : QR okundu bilgisi
 - KAMI_PITCH_MIN   : Dalış kontrolcüsü minimum pitch degeri
*/


#include "mode.h"
#include "Plane.h"

int8_t kamikaze_state = 0;           // 0 = arayis , 1 = dalis , 2 = kalkis
int8_t kamikaze_prev_state = 0;      // onceki dongudeki kami durumunu tutar
int32_t previous_pitch_limit = 0;    // tecs kontrolcüsünün minimum pitch limitini saklamak için 
int32_t yonelim_error = 0;           // aracın hedef ile arasındaki heading farkını tutmak için
int32_t dalis_yaricap = 0;           // irtifa ve açıyla hesaplanan yarıçapın tutulması için
int32_t dalis_pitch_cd = 0;          // istenilen dalış rota açısı
Location kamikaze_target;            // gerçek kamikaze hedefinin tutulduğu değişken

// next waypointin tutulacagi degiskenler
int latitude;
int longitude;

// parametrelerin degerlerinin tutulacagi degiskenler
int32_t lat1;
int32_t lat2;
int32_t lon1;
int32_t lon2;


int8_t rise_basla = 0;               // cikis waypointini bir kere tanimlamak icin          
float actual_dive_angle = 0;         // aracın anlık olduğu konum ile hedef arasındaki anlık açı
float dive_error = 0;                // aracın anlık olduğu konum ile hedef aci arasındaki fark


float integrator = 0.0f;
float output_P = 0.0f;
float dt = 0.0f;
float imax = 5.0f; // integrator max degeri (derece)
int rate = 0; // dongu hızı (hz)

void ModeKamikaze::dive()
{ // YENI PI DALIS KODU

    kamikaze_state = 1;

    // dalis acisinin altina inerse rise moduna girmesin diye dalis capi artirilir.
    dalis_yaricap = 10000;

    // bir kere yapsın diye
    if (kamikaze_prev_state != kamikaze_state)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "--kbas");
    }

    //  (0-90 derece araliginda float)  gerçekleşen dalış açısı anlık olarak hedef olan tanjantla alınır.
    actual_dive_angle = RAD_TO_DEG * atanf((plane.relative_altitude / plane.current_loc.get_distance(kamikaze_target)));

    //onceki dongu ile arasinda gecen sure (saniye cinsinden float)
    dt = plane.scheduler.get_last_loop_time_s();
    rate = plane.scheduler.get_loop_rate_hz();

    // float degree cinsinden dalış hatası gerçekleşen dalış açısı ile istenen arasındaki farkı alır
    dive_error = actual_dive_angle - plane.g.kamikaze_dive_angle;

    // PI controller 
    integrator += plane.g.kamikaze_controller_i * dive_error * dt;
    integrator = constrain_float(integrator, 0, imax);
    output_P = plane.g.kamikaze_controller_p * dive_error;

    dalis_pitch_cd = 100.0f * (output_P + integrator + plane.g.kamikaze_dive_angle);

    //gcs().send_text(MAV_SEVERITY_INFO, "dalis aci farki: %f dalis_pitch_cd: %ld, actual_dive_angle: %f", -dive_error, dalis_pitch_cd, actual_dive_angle);
    //gcs().send_text(MAV_SEVERITY_INFO, "dt: %f ,  hz= %d", dt, rate);
    gcs().send_text(MAV_SEVERITY_INFO, "p output: %f,  i output: %f", output_P, integrator);



    // hedef pitche izin vermesi için sınır düşürülür
    plane.pitch_limit_min_cd = -(dalis_pitch_cd + 1);

    // ardupilotun attitude PID kontrolcüsüne dalış anında istenilen roll ve pitch değerleri gönderilir.
    // plane.nav_pitch_cd = -dalis_pitch_cd ;
    plane.nav_pitch_cd = -1 * constrain_int32(dalis_pitch_cd, 0, plane.g.kamikaze_pitch_min);
    plane.nav_roll_cd = 0;

    // dalis sirasinda motoru kapatir
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);

    // kamikaze durumu dalış olarak tanımlanır.
}

/*                              ESKI CALISAN DIVE KODU
void ModeKamikaze::dive()
{

        kamikaze_state = 1;

        //bir kere yapsın diye
        if(kamikaze_prev_state != kamikaze_state){
            gcs().send_text(MAV_SEVERITY_INFO,"--kbas");
        }        

        // gerçekleşen dalış açısı anlık olarak hedef olan tanjantla alınır.
        actual_dive_angle = RAD_TO_DEG * atanf((plane.relative_altitude / plane.current_loc.get_distance(kamikaze_target)));
        
        // dalış hatası gerçekleşen dalış açısı ile istenen arasındaki farkı alır
        dive_error = actual_dive_angle*100 - plane.g.kamikaze_dive_angle;

        // istenilen pitch açısı, istenilen dalış açısının üstüne hata miktarı eklenerek hesaplanır. hata miktarı kontrolcü katsayısı ile çarpılır.
        dalis_pitch_cd = plane.g.kamikaze_dive_angle + (dive_error * plane.g.kamikaze_controller_p); 
        
        gcs().send_text(MAV_SEVERITY_INFO,"dalis aci farki: %d, dalis_pitch_cd: %d, actual_dive_angle: %f", -dive_error, dalis_pitch_cd, actual_dive_angle); 

        // hedef pitche izin vermesi için sınır düşürülür
        plane.pitch_limit_min_cd = -(dalis_pitch_cd+1); 
        
        // PID kontrolcüsüne dalış anında istenilen roll ve pitch değerleri gönderilir.
        //plane.nav_pitch_cd = -dalis_pitch_cd ;
        plane.nav_pitch_cd = -1*constrain_int32(dalis_pitch_cd, 0, plane.g.kamikaze_pitch_min);
        plane.nav_roll_cd = 0;

        // dalis sirasinda motoru kapatir
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);

        //kamikaze durumu dalış olarak tanımlanır.
         
}
*/

/*
void ModeKamikaze::rise()
{
        // araç hedefi geçtikten sonra da başka yöne yönelmeden yükselmeye devam edebilmesi ve level seviyesinde moddan çıkabilmesi için aracın 1000m ilerisine geçici bir waypoint atanır.
        plane.next_WP_loc = plane.current_loc;
        plane.next_WP_loc.offset_bearing(plane.gps.ground_course_cd()*0.01f, 1000); 
               
        // PID kontrolcüsüne kalkış manevrasında yapması istenen roll ve pitch değerleri gönderilir.
        plane.nav_pitch_cd = plane.g.kamikaze_rise_angle*100; // yükseliş pitch KAMI_RISE_ANGLE parametresinden çekilir. degree -> centidegree dönüşümü yapılır.
        plane.nav_roll_cd = 0;
        plane.calc_throttle();

        //kamikaze durumu kalkış olarak tanımlanır.
        kamikaze_state = 2;

}
*/
void ModeKamikaze::rise()
{

    // kalkis basladiginda 500m ileriye wp atilir. wp'nin yuksekligi kami alt sinirinin 10m fazlasi olur
    if(rise_basla == 0){
        plane.prev_WP_loc = plane.current_loc;
        plane.next_WP_loc.offset_bearing(plane.gps.ground_course_cd()*0.01f, 500); 
        plane.next_WP_loc.alt = 10 + plane.g.kamikaze_end_altitude*100 + AP::ahrs().get_home().alt; // end altitude + 10metre
        plane.set_target_location(plane.next_WP_loc);
        kamikaze_state = 2;
        if(kamikaze_prev_state != kamikaze_state){
            gcs().send_text(MAV_SEVERITY_INFO, "--kbit");
        }  
    }
    
    if(plane.relative_altitude > plane.g.kamikaze_end_altitude){
        plane.set_mode(plane.mode_rtl,ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO,"UPDATE Mod Degis");
    }
    
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle(); 

    rise_basla = 1;        
}

void ModeKamikaze::search()
{
        // navigate() fonksiyonunu çalıştırır, guided davranışını devam ettirir. 
        //////   UPDATE ICINE ALINDI //////
        
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle(); 

        // kamikaze durumu arayış olarak tanımlanır.
        kamikaze_state = 0;

}

bool ModeKamikaze::_enter()
{
    

    plane.guided_throttle_passthru = false;

    Location loc{plane.current_loc};

    // reset qr_detect parameter
    plane.g.kamikaze_qr_detect.set(0);

    // reset controllers
    integrator = 0;
    output_P = 0;


    // kamikaze hedefi moda girildiğindeki waypoint olarak atanır
    // parametrelerdeki degerleri ceker
    lat1 = plane.g.kamikaze_lat_bas;
    lat2 = plane.g.kamikaze_lat_son;
    lon1 = plane.g.kamikaze_lon_bas;
    lon2 = plane.g.kamikaze_lon_son;

// Ardupilot location classinda konumu 1e7 ile carpip integer olarak kullanir.
    latitude = (lat1 * 100000) + lat2;
    longitude = (lon1 * 100000) + lon2;

    plane.prev_WP_loc = plane.current_loc;

    plane.next_WP_loc.lat = latitude;
    plane.next_WP_loc.lng = longitude;
    plane.next_WP_loc.alt = plane.g.kamikaze_start_alt*100 + AP::ahrs().get_home().alt;

    kamikaze_target = plane.next_WP_loc;

    kamikaze_state = 0; // kamikaze durumu default haliyle arayış olarak tanımlanır 
    previous_pitch_limit = plane.pitch_limit_min_cd; // tecs kontrolcüsü minimum pitch limiti dalış sırasında değiştirileceği için eski değer hafızaya atılır.

    rise_basla = 0;               // cikis waypointini bir kere tanimlamak icin 

    // burasi autodan alinma , autoya gectiginde kaldigi yerden devam etmesi icin
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();
    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }


    gcs().send_text(MAV_SEVERITY_INFO,"Kamikaze Enter"); 
    //gcs().send_text(MAV_SEVERITY_INFO,"lat: %d, long: %d",latitude,longitude);

    return true;
}

void ModeKamikaze::update()
{
    
    // hedef ile uçağın baş açısı arasındaki fark ölçülür
    // get_bearing, hedefin araca göre kaç derece yönünde olduğunu centidegree cinsinden verir.
    yonelim_error = abs( plane.ahrs.yaw_sensor - plane.current_loc.get_bearing_to(kamikaze_target) );
    
    // istenilen dalış açısına göre irtifa hesaba katılarak dalışa başlanması gereken mesafe bulunur
    dalis_yaricap = plane.relative_altitude / (tanF(plane.g.kamikaze_dive_angle * DEG_TO_RAD));
    //gcs().send_text(MAV_SEVERITY_INFO,"dalis yaricap yeni %ld",dalis_yaricap);


    // start altitude degerini mod icinde degistirmeye imkan saglar
    plane.next_WP_loc.alt = plane.g.kamikaze_start_alt*100 + AP::ahrs().get_home().alt;
    kamikaze_target = plane.next_WP_loc;
    

    //araç, dokümantasyonda belirtilen şekilde hedefin neresinde olduğuna göre yapacağı hareketi seçer.

    if(rise_basla == 1 && plane.relative_altitude > plane.g.kamikaze_end_altitude)
    {
        plane.set_mode(plane.mode_auto,ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO,"Kamikaze Yukselis Sonlandı Mod Degisiyor");
    }

    if(plane.current_loc.get_distance(kamikaze_target) <= (dalis_yaricap))
    {
        if( plane.g.kamikaze_qr_detect == 0 && plane.relative_altitude >= plane.g.kamikaze_end_altitude)
        {
            if((yonelim_error <= 1500 || yonelim_error >= 34500) && abs(plane.ahrs.roll_sensor) < 750 )
            {
                dive();
            }
            else
            {
                if(yonelim_error > 10000 && yonelim_error < 26000)
                {
                    plane.set_mode(plane.mode_auto,ModeReason::MISSION_END);
                    gcs().send_text(MAV_SEVERITY_INFO,"Hedef Arkada Mod Degis");
                }
                else
                {
                    search();
                }
            }
        }
        else
        {
            rise();
        }

    }
    else
    {
        search();
    }

    if(kamikaze_prev_state != kamikaze_state){
        gcs().send_text(MAV_SEVERITY_INFO,"mode_state: %d",kamikaze_state);
    }
    kamikaze_prev_state = kamikaze_state;


}

void ModeKamikaze::navigate()
{
    // hedefin etrafından dönme eğilimini engellemek için guided modu içerisindeki update_loiter komutu kaldırılmış yerine waypoint kontrolcüsü eklenmiştir.
    // waypoint pathinde A noktası aracın anlık konumu, B noktası next_wp olarak tanımlanır. wp genişliği 0'a yakın atanır.
    plane.nav_controller->update_waypoint(plane.current_loc,plane.next_WP_loc,0.01f);
}


void ModeKamikaze::_exit()
{
    // moddan çıkılırken tüm değişkenler sıfırlanır ve moddan çıkıldığı GCS'ye yazdırılır.
    kamikaze_state = 0;
    yonelim_error = 0;
    dalis_yaricap = 0;
    dalis_pitch_cd = 0;
    plane.pitch_limit_min_cd = previous_pitch_limit;
    rise_basla = 0;           
    actual_dive_angle = 0;       
    dive_error = 0;
    plane.g.kamikaze_qr_detect.set(0);
    gcs().send_text(MAV_SEVERITY_INFO,"Kamikaze Exit");
}

bool ModeKamikaze::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

bool ModeKamikaze::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}
