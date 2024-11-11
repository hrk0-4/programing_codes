#include<math.h>
//ステアリングの右回りピンを設定
#define steering_motorR_Pin 0
//ステアリングの左回りピンを設定
#define steering_motorL_Pin 2
//ステアリングのPWM制御用ピンを設定
#define steering_motorPWM_Pin 9
//ホイールの正転ピン
#define wheel_motorF_Pin
//ホイールの反転ピン
#define wheel_motorR_Pin
//ホイールのPWM制御ピン
#define wheel_motorPWM_Pin
// ポテンショメータ（位置センサー）を接続するピンを指定します
#define potPin 4
//PWMの周波数を設定
const int pulse_freq = 1000;
//PWMの解像度（分解能）をBits で設定（この場合[0,255]がPWMの範囲）
const char pulse_resolution_bits = 8;
//速度の最大値を設定（この場合スティック入力を仮定した数値）
const double max_velosity = sqrt(120*120 + 120*120);

//直交座標(x,y)または極座標(r,angle)を入れる構造体を宣言
struct coordinate{
  double a;
  double b;
};
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
void setup() {
  //シリアル通信の周波数を設定
  Serial.begin(9600);
  Serial.printf("Ready.\n");
  // ステアリング用モーターの右回りピンを出力に設定します
  pinMode(steering_motorR_Pin, OUTPUT);
  //ステアリング用モーターの左回りピンを出力に設定します
  pinMode(steering_motorL_Pin, OUTPUT);
  //ステアリング用モーターのPWMピンの設定します
  ledcAttach(steering_motorPWM_Pin, pulse_freq, pulse_resolution_bits);
  //ホイール用モータの正転ピンを出力
  pinMode(wheel_motorF_Pin,OUTPUT);
  //ホイール用モータの反転ピンを出力
  pinMode(wheel_motorR_Pin,OUTPUT);
  //ホイール用モータのPWMピン,周波数,分解能を設定する
  ledAttach(wheel_motorPWM_Pin, pluse_freq, pulse_resolution_bits);
  //ホイールの初期位置をポテンショメータから読み取ります（この位置を0°とする）
  const int original_position = analogRead(potPin);
}

void loop() {
  //ホイールの初期位置をポテンショメータから読み取る（この位置を0°とする）
  const int original_position = analogRead(potPin);//これいいのか？
  double x = 100, y = 100;
  //ステアリングの直交座標上の目標方向の座標または各成分の速度ベクトル(x,y)
  coordinate target_xy = {x,y};
  //ステアリングの目標方向を極座標に変換(r = target_polar.a, angle = target_polar.b)
  coordinate target_polar = {convert_xy_to_polar(target_xy)};
  Serial.printf("r = %f, angle = %f\n",target_polar.a, target_polar.b);
  //目標方向の角度angle[-π,π]の値を扱いやすくするため仮想的な範囲[-512,512]上の値に変換
  int target_position_in_virtual_range = target_polar.b / (2 * M_PI) * 1024;
  Serial.printf("target_position:%d\n",target_position_in_virtual_range);
  //現在のホイールの正転方向の傾きの値[0,1023]をセンサーからそのまま入れる
  int current_position_in_original_range = analogRead(potPin);
  //現在のホイールの正転方向の傾き[0,1023]の値を扱いやすくするため仮想的な範囲[-512,512]上の値に変換
  int current_position_in_virtual_range = slide_range(current_position_in_original_range, original_position);
  Serial.printf("%d → %d\n",current_position_in_original_range, current_position_in_virtual_range);
  //各positionの値が負の時+512することで[0,512]で扱えるようにする
  if(target_position_in_virtual_range < 0)
    int target_position = 512 + target_position_in_virtual_range;
  if(current_position_in_virtual_range < 0)
    int current_position = 512 + current_position_in_virtual_range;
  Serial.printf("|target| = %d, |current| = %d\n", target_position, current_position);
  //[0,512]内の目標位置と現在位置の差difference(-512,512)を算出
  int difference = current_position - target_position;
  //差の絶対値が42(約15°)より大きいときのみステアリング
  if(fabs(difference) > 42){
    //|difference|または512-|difference|（一方が必ず[0,255]）の小さい方を256を最大値とした割合として扱えるようにしてtarget_outputに入れる
    double target_output_of_steering = fmin(fabs(difference), fabs(512 - fabs(difference))) / 256;
    //急激な電流の変化を防ぐため出力を徐々にあげる
    double current_output_of_steering = control_pwm_output(target_output_of_steering, current_output_of_steering);
    //
    if(fabs(difference) < 256){
      //|difference|<256（つまり90°未満）ならばdifferenceの正負によってホイールの傾きと目標方向との位置関係がわかり,回転方向が一意的に決まる
      digitalWrite(steering_motorR_Pin,(difference) < 0);
      digitalWrite(steering_motorL_Pin,(difference) > 0);
      ledcWrite(steering_motorPWM_Pin,int(current_output_of_steering * 255));
      //上記と同様に考えれる。
    }else{
      digitalWrite(steering_motorR_Pin,(difference) > 0);
      digitalWrite(steering_motorL_Pin,(difference) < 0);
      ledcWrite(steering_motorPWM_Pin,int(current_output_of_steering * 255));
    }
  //|difference|<42ならば角度調節を終了し,走行に移る
  }else{
    //角度調節を終わらせる
    ledcWrite(steering_motorPWM_Pin,0);
    Serial.printf("%d  %d\n", current_position, target_position);
    //仮想的な範囲(-512,512]上での差の大きさをとる
    int difference_in_virtual_range = fabs(target_position_in_virtual_range - current_position_in_virtual_range);
    //差が768より大きい（または256未満）の場合,目標方向のベクトルとホイールの正転方向のベクトルが一致している（ほぼ）
    if(difference_in_virtual_range > 768){
      differance_in_vitual_range = 1024 - difference_in_virtual_range;
    }
    //駆動のモータの出力をベクトルの大きさrを元に考える,その際rの最大値で割って割合として扱う
    double target_output_of_wheel = target_polar.a / max_velosity;
    //徐々に出力をあげる
    double current_output_of_wheel = contol_pwm_output(target_output_of_wheel, current_output_of_wheel);
    //正転方向にまわす
    if(difference_in_virtual_range < 256){
      digitalWrite(wheel_motorF_Pin,HIGH);
      digitalWrite(wheel_motorR_Pin,LOW);
      ledcWrite(wheel_motorPWM_Pin,int(current_output_of_wheel * max_velosity));
     //反転方向に回す 
    }else if(256 <= difference_in_virtual_range && difference_in_virtual_range < 768){
      digitalWrite(wheel_motorF_Pin,LOW);
      digitalWrite(wheel_motorR_Pin,HIGH);
      ledcWrite(wheel_motorPWM_Pin,int(current_output_of_wheel * max_velosity));
    }
  }
  
  // 0.1秒待ちます
  delay(100);
}