/*
 * This file can be used to define custom register addresses
 * Just use addresses 100 and above
 */
 
#define REG_COUNTER_VALUE_L    100
#define REG_COUNTER_VALUE_H    101
#define REG_COUNTER_DIRECTION  102

unsigned char lift_direction;
int lift_counter;
void userSetup()
{
  PCICR |= (1 << PCIE0);  // enable PC interrupt on port A
  PCMSK0 |= 1;            // enable interrupt A0
  lift_direction = 0;
  lift_counter = 0;
}

ISR(PCINT0_vect){ 
  if(lift_direction > 0)
    lift_counter++;
  else
    lift_counter--;
}

unsigned char userWrite(int addr, unsigned char param)
{
  if(addr == REG_COUNTER_DIRECTION){
    lift_direction = param;
  }else if(addr == REG_COUNTER_VALUE_L){
    lift_counter = (lift_counter&0xff00)+param;
  }else if(addr == REG_COUNTER_VALUE_H){
    lift_counter = (lift_counter&0x00ff)+(param<<8);
  }else{
    return ERR_INSTRUCTION;
  }
  return ERR_NONE;
}

/* Read one byte from register located at addr */
int userRead(int addr)
{
  if(addr == REG_COUNTER_DIRECTION){
    return lift_direction;    
  }else if(addr == REG_COUNTER_VALUE_L){
    return lift_counter%256;
  }else if(addr == REG_COUNTER_VALUE_H){
    return ((unsigned int)lift_counter>>8)%256;
  }
  return 0; 
}
