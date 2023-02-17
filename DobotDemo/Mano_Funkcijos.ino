void moveArm(float x, float y, float z, float r)
{
  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  gPTPCmd.r = r;

  Serial.print("move to x:"); Serial.print(gPTPCmd.x); Serial.print(" y:"); Serial.print(gPTPCmd.y); Serial.print(" z:"); Serial.println(gPTPCmd.r);

  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  ProtocolProcess();

  currentX = x;
  currentY = y;
  currentZ = z;
  currentR = r;
}

void Griperis_suspausti(){
   SetEndEffectorGripper(true, true, &gQueuedCmdIndex);
   ProtocolProcess();
   delay(2000);
   suspausta == true;
   
}

void Griperis_atleisti(){
    if(suspausta == true){
      SetEndEffectorGripper(true, false, &gQueuedCmdIndex);  // iskleidziam bet oras dar ijungtas
      ProtocolProcess();
      delay(2000);
      suspausta == false;
    }
    SetEndEffectorGripper(false, true, &gQueuedCmdIndex);
    ProtocolProcess();
    suspausta == false;
    delay(2000);
}

void Siurbtuka(bool state){
  SetEndEffectorSuctionCup(state, false, &gQueuedCmdIndex);
  ProtocolProcess();
  delay(2000);
}

//////////////////////////////////////////////////////////////////////////////////////

void SetHOMEParams(float x, float y, float z, byte cmd) {
  byte cmd_set_home_params[kDataLength] = {0};
  cmd_set_home_params[0] = kDataLength;
  cmd_set_home_params[1] = 0x00;
  cmd_set_home_params[2] = 0x00;
  cmd_set_home_params[3] = 0x00;
  cmd_set_home_params[4] = 0x00;
  cmd_set_home_params[5] = kCmdSetHOMEParams;
  memcpy(&cmd_set_home_params[6], &x, sizeof(float));
  memcpy(&cmd_set_home_params[10], &y, sizeof(float));
  memcpy(&cmd_set_home_params[14], &z, sizeof(float));
  cmd_set_home_params[18] = cmd;

  Serial1.write(cmd_set_home_params, kDataLength);
}

void Home() {
  float x = 200.0;
  float y = 0.0;
  float z = 0.0;
  byte cmd = 0;
  SetHOMEParams(x, y, z, cmd);

  byte cmd_home[7] = {0};
  cmd_home[0] = 7;
  cmd_home[1] = 0x00;
  cmd_home[2] = 0x00;
  cmd_home[3] = 0x00;
  cmd_home[4] = 0x00;
  cmd_home[5] = kCmdHOMECmd;
  cmd_home[6] = 1;

  Serial1.write(cmd_home, 7);
}

