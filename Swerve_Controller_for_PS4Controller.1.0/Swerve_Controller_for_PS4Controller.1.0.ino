#include<math.h>
#include<PS4Controller.h>
  //座標(x.y)を入れる構造体を宣言
  struct coordinate{
    double a;
    double b;
  };

  //ホイールの数を入力
  const int Number_of_Wheel = 4;
  //ステアリング用モータ右回転ピンを設定
  const char steering_motorR_Pin[] = {};
  //ステアリング用モータ左回転ピンを設定
  const char steering_motorL_Pin[] = {};
  //駆動用モータ正回転ピンを設定
  const char wheel_motorF_Pin[] = {};
  //駆動用モータ反回転ピンを設定
  const char wheel_motorR_Pin[] = {};
  //駆動用モータのスピード制御ピンを設定
  const char wheel_motorPWM_Pin[] = {};
  //ステアリングのスピード制御ピンを設定
  const char steering_motorPWM_Pin[] = {};
  //ポテンショメータ(位置センサー)を接続するピンを設定
  const char position_Pin[] = {};
  //PWMの周波数を設定
  const int pulse_freq = 1000;
  //PWMの解像度(分解能)をBitsで設定
  const char pulse_resolution_bits = 8;
  //スティックの入力の絶対値を設定
  const double max_velosity = sqrt(120*120 + 120*120);
  //直交座標系を極座標系に変換する関数
  coordinate convert_xy_to_polar(coordinate xy){
    double r, angle;
    r = sqrt(xy.a * xy.a + xy.b * xy.b);
    angle = atan2(xy.b, xy.a);
    return{r, angle};
  };
//センサーがとる値を[0,1023]から[-512,512]に変換する関数
int slide_range(int pre_position, int original_position){
  //（以下、original_position = 900として書く）
  //基準となる初期位置が512より大きい場合
  if(original_position > 512){
    //[900,1023]にある値を[0,123]に変換（例えば1000を100に）
    if(pre_position >= original_position){
      //その差を返す
      return(pre_position - original_position);
    //[0,388]の範囲にある値を[124,512]に変換（例えば224を348に）
    }else if(0 <= pre_position && pre_position <= original_position - 512){
      return(1024 - (original_position - pre_position));
      //[389,899]を[-511,-1]に
    }else{
      return(pre_position - original_position);
    }
  //（以下,original_position = 200として書く）
  //初期位置が512以下の場合
  }else{
    //[200,712]to[0,512]
    if(original_position <= pre_position && pre_position <= original_position + 512){
      return(pre_position - original_position);
    //[0,199]to[-200,-1]
    }else if(pre_position < original_position){
      return(pre_position - original_position);
    //[713,1023]to[-511,-201]
    }else{
      return(-(1024 - pre_position + original_position));
    }
  }
};
//大電流をいきなり流さないため,徐々にアゲテく関数
double control_pwm_output(double target_output, double current_output){
  if(target_output > current_output){
    current_output += 0.1; 
    return(current_output);
  }else{
    return(target_output);
  }
}
//ホイールの現在の傾きを[0,1023]でセンサーから受け取る変数
int current_position_in_original_range[Number_of_Wheel];
//ホイールの現在の傾きを[-512,512]で表した変数
int current_position_in_virtual_range[Number_of_Wheel];
//
int Number_of_adjustedWheel;
//ホイールの初期角度
int original_position[Number_of_Wheel];
int difference_in_virtual_range[Number_of_Wheel];
int difference[Number_of_Wheel];
int target_position_in_virtual_range;
double target_output_of_steering[Number_of_Wheel];
double current_output_of_steering[Number_of_Wheel];
double target_output_of_wheel;
double current_output_of_wheel;
double target_angle_of_turning[Number_of_Wheel] = {-3*M_PI/4, -M_PI/4, M_PI/4, 3*M_PI/4};

void setup() {
  //シリアル通信の周波数を設定
  Serial.begin(9600);
  //PS4コントローラのアドレスを指定し接続
  PS4.begin("");
  Serial.println("Ready.");
  for(int i = 0; i < Number_of_Wheel; i++){
    //ステアリング用モータ右回転ピンを出力に設定
    pinMode(steering_motorR_Pin[i], OUTPUT);
    //ステアリング用モータ左回転ピンを出力に設定
    pinMode(steering_motorL_Pin[i], OUTPUT);
    //ステアリング用モータPWMピンの(ピン番号,周波数,分解能)を設定
    ledcAttach(steering_motorPWM_Pin[i], pulse_freq, pulse_resolution_bits);
    //駆動用モータ正回転ピンを出力に設定
    pinMode(wheel_motorF_Pin[i], OUTPUT);
    //駆動用モータ反回転ピンを出力に設定
    pinMode(wheel_motorR_Pin[i], OUTPUT);
    //駆動用モータPWMピンの設定
    ledcAttach(wheel_motorPWM_Pin[i], pulse_freq, pulse_resolution_bits);
  } 
}

void loop() {
  for(int i = 0; i < Number_of_Wheel; i++ ){
    original_position[i] = analogRead(position_Pin[i]);
  }
  Number_of_adjustedWheel = 1;

  if(PS4.isConnected()){
    //旋回する場合
    if(PS4.L2() || PS4.R2()){
      if(PS4.L2()){
        for(int i; i < Number_of_Wheel; i++){
          //現在のホイールの正転方向の傾きの値[0,1023]をセンサーからそのまま入れる
          current_position_in_original_range[i] = analogRead(position_Pin[i]);
          //現在のホイールの正転方向の傾き[0,1023]の値を扱いやすくするため仮想的な範囲[-512,512]上の値に変換
          current_position_in_virtual_range[i] = slide_range(current_position_in_original_range[i], original_position[i]);
          Serial.printf("%d → %d\n",current_position_in_original_range[i], current_position_in_virtual_range[i]);
          target_angle_of_turning[i] = target_angle_of_turning[i] / (2 * M_PI) * 1024;
          Serial.printf("%f\n",target_angle_of_turning[i]);
          //仮想的な範囲(-512,512]上での差の大きさをとる
          difference_in_virtual_range[i] = fabs(target_angle_of_turning[i] - current_position_in_virtual_range[i]);
          //差が768より大きい（または256未満）の場合,目標方向のベクトルとホイールの正転方向のベクトルが一致している（ほぼ）
          if(difference_in_virtual_range[i] > 768)
            difference_in_virtual_range[i] = 1024 - difference_in_virtual_range[i];
          //ステアリングの最小角を調べるため各positionの値が負の時+512することで[0,512]で扱えるようにする
          if(target_angle_of_turning[i] < 0)
            target_angle_of_turning[i] = 512 + target_angle_of_turning[i];
          //各positionの値が負の時+512することで[0,512]で扱えるようにする
          if(current_position_in_virtual_range[i] < 0)
            current_position_in_virtual_range[i] = 512 + current_position_in_virtual_range[i];
          Serial.printf("|target| = %d, |current| = %d\n", target_angle_of_turning[i], current_position_in_virtual_range[i]);
          //[0,512]内の目標位置と現在位置の差difference(-512,512)を算出
          difference[i] = current_position_in_virtual_range[i] - target_angle_of_turning[i];
        }
        for(int i = 0; i < Number_of_Wheel; i++){
          if(fabs(difference[i]) > 42){
            //|difference|または512-|difference|（一方が必ず[0,255]）の小さい方を256を最大値とした割合として扱えるようにしてtarget_outputに入れる
            target_output_of_steering[i] = fmin(fabs(difference[i]), fabs(512 - fabs(difference[i]))) / 256;
            //急激な電流の変化を防ぐため出力を徐々にあげる
            current_output_of_steering[i] = control_pwm_output(target_output_of_steering[i], current_output_of_steering[i]);
            //
            if(fabs(difference[i]) < 256){
              //|difference|<256（つまり90°未満）ならばdifferenceの正負によってホイールの傾きと目標方向との位置関係がわかり,回転方向が一意的に決まる
              digitalWrite(steering_motorR_Pin[i],(difference[i]) < 0);
              digitalWrite(steering_motorL_Pin[i],(difference[i]) > 0);
              ledcWrite(steering_motorPWM_Pin[i],int(current_output_of_steering[i] * 255));
              //上記と同様に考えれる。
            }else{
              digitalWrite(steering_motorR_Pin[i],(difference[i]) > 0);
              digitalWrite(steering_motorL_Pin[i],(difference[i]) < 0);
              ledcWrite(steering_motorPWM_Pin[i],int(current_output_of_steering[i] * 255));
            }
          }
        }
        //|difference|<42ならば角度調節を終了し,旋回に移る
        for(int i = 0; i < Number_of_Wheel; i++){
          if(fabs(difference[i]) < 42)
            Number_of_adjustedWheel += 1;
        }
        if(Number_of_adjustedWheel == 4){
          for(int i = 0; i < Number_of_Wheel; i++){
            //角度調節を終わらせる
            ledcWrite(steering_motorPWM_Pin[i],0);
            Serial.printf("%d  %d\n", current_position_in_virtual_range[i], target_position_in_virtual_range);
          }
          for(int i = 0; i < Number_of_Wheel; i++){
            //駆動のモータの出力をベクトルの大きさrを元に考える,その際rの最大値で割って割合として扱う
            target_output_of_wheel = max_velosity;
            //徐々に出力をあげる
            current_output_of_wheel = control_pwm_output(target_output_of_wheel, current_output_of_wheel);
            //正転方向にまわす
            if(difference_in_virtual_range[i] < 256){
              digitalWrite(wheel_motorF_Pin[i],HIGH);
              digitalWrite(wheel_motorR_Pin[i],LOW);
              ledcWrite(wheel_motorPWM_Pin[i],int(current_output_of_wheel * max_velosity));
            //反転方向に回す 
            }else if(256 <= difference_in_virtual_range[i] && difference_in_virtual_range[i] < 768){
              digitalWrite(wheel_motorF_Pin[i],LOW);
              digitalWrite(wheel_motorR_Pin[i],HIGH);
              ledcWrite(wheel_motorPWM_Pin[i],int(current_output_of_wheel * max_velosity));
            }
          }
        } 
      }
    }
    //機体の直交座標上の目標方向の座標,各成分の速度ベクトル(x,y)
    coordinate target_xy;
    //PS4.LStickX, PS4.LStickYの値にそれぞれ0.5を足し正負の値を等価にして変数target_xyに代入
    target_xy = {PS4.LStickX() + 0.5, PS4.LStickY() + 0.5};
    //コントローラのデッドゾーン設定
    //target_xy.x絶対値が7.5未満であれば0にし,7.5以上であれば絶対値から7.5を引いたものを符号を戻して代入
    target_xy.a -= (target_xy.a/fabs(target_xy.a))*fmin(fabs(target_xy.a), 7.5);
    //yも同様に
    target_xy.b -= (target_xy.b/fabs(target_xy.b))*fmin(fabs(target_xy.b), 7.5);
    //各速度を表示
    Serial.printf("Vx:%lf, Vy:%lf\n",target_xy.a, target_xy.b);
    //ステアリングの目標方向を極座標に変換(r = target_polar.a, angle = target_polar.b)
    coordinate target_polar = {convert_xy_to_polar(target_xy)};
    Serial.printf("r = %f, angle = %f\n",target_polar.a, target_polar.b);
    //目標方向の角度angle[-π,π]の値を扱いやすくするため仮想的な範囲[-512,512]上の値に変換
    target_position_in_virtual_range = target_polar.b / (2 * M_PI) * 1024;
    Serial.printf("target_position:%d\n",target_position_in_virtual_range);
    for(int i = 0; i < Number_of_Wheel; i++){
      //現在のホイールの正転方向の傾きの値[0,1023]をセンサーからそのまま入れる
      current_position_in_original_range[i] = analogRead(position_Pin[i]);
      //現在のホイールの正転方向の傾き[0,1023]の値を扱いやすくするため仮想的な範囲[-512,512]上の値に変換
      current_position_in_virtual_range[i] = slide_range(current_position_in_original_range[i], original_position[i]);
      Serial.printf("%d → %d\n",current_position_in_original_range[i], current_position_in_virtual_range[i]);
      //仮想的な範囲(-512,512]上での差の大きさをとる
      difference_in_virtual_range[i] = fabs(target_position_in_virtual_range - current_position_in_virtual_range[i]);
      //差が768より大きい（または256未満）の場合,目標方向のベクトルとホイールの正転方向のベクトルが一致している（ほぼ）
      if(difference_in_virtual_range[i] > 768)
        difference_in_virtual_range[i] = 1024 - difference_in_virtual_range[i];
    }
    //ステアリングの最小角を調べるため各positionの値が負の時+512することで[0,512]で扱えるようにする
    if(target_position_in_virtual_range < 0)
      target_position_in_virtual_range = 512 + target_position_in_virtual_range;

    for(int i = 0; i < Number_of_Wheel; i++){
      //各positionの値が負の時+512することで[0,512]で扱えるようにする
      if(current_position_in_virtual_range[i] < 0)
        current_position_in_virtual_range[i] = 512 + current_position_in_virtual_range[i];
      Serial.printf("|target| = %d, |current| = %d\n", target_position_in_virtual_range, current_position_in_virtual_range[i]);
      //[0,512]内の目標位置と現在位置の差difference(-512,512)を算出
      difference[i] = current_position_in_virtual_range[i] - target_position_in_virtual_range;
    }

    //差の絶対値が42(約15°)より大きいときのみステアリング
    for(int i = 0; i < Number_of_Wheel; i++){
      if(fabs(difference[i]) > 42){
        //|difference|または512-|difference|（一方が必ず[0,255]）の小さい方を256を最大値とした割合として扱えるようにしてtarget_outputに入れる
        target_output_of_steering[i] = fmin(fabs(difference[i]), fabs(512 - fabs(difference[i]))) / 256;
        //急激な電流の変化を防ぐため出力を徐々にあげる
        current_output_of_steering[i] = control_pwm_output(target_output_of_steering[i], current_output_of_steering[i]);
        //
        if(fabs(difference[i]) < 256){
          //|difference|<256（つまり90°未満）ならばdifferenceの正負によってホイールの傾きと目標方向との位置関係がわかり,回転方向が一意的に決まる
          digitalWrite(steering_motorR_Pin[i],(difference[i]) < 0);
          digitalWrite(steering_motorL_Pin[i],(difference[i]) > 0);
          ledcWrite(steering_motorPWM_Pin[i],int(current_output_of_steering[i] * 255));
          //上記と同様に考えれる。
        }else{
          digitalWrite(steering_motorR_Pin[i],(difference[i]) > 0);
          digitalWrite(steering_motorL_Pin[i],(difference[i]) < 0);
          ledcWrite(steering_motorPWM_Pin[i],int(current_output_of_steering[i] * 255));
        }
      }
    }

    //|difference|<42ならば角度調節を終了し,走行に移る
    for(int i = 0; i < Number_of_Wheel; i++){
      if(fabs(difference[i]) < 42)
      Number_of_adjustedWheel += 1;
    }
    if(Number_of_adjustedWheel == 4){
      for(int i = 0; i < Number_of_Wheel; i++){
        //角度調節を終わらせる
        ledcWrite(steering_motorPWM_Pin[i],0);
        Serial.printf("%d  %d\n", current_position_in_virtual_range[i], target_position_in_virtual_range);
      }
      for(int i = 0; i < Number_of_Wheel; i++){
        //駆動のモータの出力をベクトルの大きさrを元に考える,その際rの最大値で割って割合として扱う
        target_output_of_wheel = target_polar.a / max_velosity;
        //徐々に出力をあげる
        current_output_of_wheel = control_pwm_output(target_output_of_wheel, current_output_of_wheel);
        //正転方向にまわす
        if(difference_in_virtual_range[i] < 256){
          digitalWrite(wheel_motorF_Pin[i],HIGH);
          digitalWrite(wheel_motorR_Pin[i],LOW);
          ledcWrite(wheel_motorPWM_Pin[i],int(current_output_of_wheel * max_velosity));
        //反転方向に回す 
        }else if(256 <= difference_in_virtual_range[i] && difference_in_virtual_range[i] < 768){
          digitalWrite(wheel_motorF_Pin[i],LOW);
          digitalWrite(wheel_motorR_Pin[i],HIGH);
          ledcWrite(wheel_motorPWM_Pin[i],int(current_output_of_wheel * max_velosity));
        }
      }
    }
  }
  
  // 0.1秒待ちます
  delay(100);
  }
