#include <Arduino.h>
#include <EEPROM.h>
#include <functional>
#include <limits.h>
#include <sstream>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "trj_loop.h"
#include "trj_messageprocessor.h"
#include "trj_fastset.h"
#include "trj_stepper.h"
#include "trj_ringbuffer.h"
#include "trj_bithacks.h"
#include "trj_config.h"
#include "trj_segment.h"

extern Loop  mainLoop;
extern StepInterface steppers[];

CurrentState current_state;


void stepISR(){ mainLoop.isr(); }

void clearISR(){ mainLoop.clearIsr();}

volatile bool finished_phase = false;

inline void Loop::isr(){

    unsigned long  steps_left = 0;
    //static bool activityToggle = true;

    if(finished_phase)
      return;

    // When we set a step, also set the ISR to clear it later. 
    clearTimer.begin(clearISR,1.55);  // 1.55 is the minimum on teensy4 == 36 cycles. 

    switch (config.n_axes) {
      //case 8: steps_left += step(7);
      //case 7: steps_left += step(6);
      case 6: steps_left += step(5);
      case 5: steps_left += step(4);
      case 4: steps_left += step(3);
      case 3: steps_left += step(2);
      case 2: steps_left += step(1);
      case 1: steps_left += step(0);
      case 0: ;
    }

    if (steps_left == 0){
      //All of the axes have finished so clear out the message 
      setTimer.end();
      finished_phase = true;
    } 
}

inline void Loop::clearIsr(){
  
  switch (config.n_axes) {
    //case 8: steppers[7]->clearStep();
    //case 7: steppers[6]->clearStep();
    case 6: steppers[5]->clearStep();
    case 5: steppers[4]->clearStep();
    case 4: steppers[3]->clearStep();
    case 3: steppers[2]->clearStep();
    case 2: steppers[1]->clearStep();
    case 1: steppers[0]->clearStep();
  }

  clearTimer.end();
}

void Loop::setup(){

  stop();
  disable();

  return;
  EEPROM.get(EEPROM_OFFSET, config);
  setConfig(&config, false);
 
  for(int i=0; i < config.n_axes; i++){
    EEPROM.get(EEPROM_OFFSET+sizeof(config)+(sizeof(AxisConfig)*i), axes_config[i]);
    setAxisConfig(&axes_config[i], false);
  }


  
}

/* Run one iteration of the main loop
*/
void Loop::loopOnce(){
    loopTick(); // Blink the LED and toggle a debugging pin, to show we're not crashed. 
    runSerial(); // Get serial data and update queues 
    feedSteppers(); // Start stepper ISRs, if there is data. 
}

/* Display operaton status on the builtin LED
*/
void Loop::loopTick()    {
    static unsigned long last = millis();
    static bool ledToggle = true;
    
    // Fast tick for running, slow for idle
    if( millis() - last > (running ? 100 : 2000)  ){
      digitalWrite(LED_BUILTIN, (ledToggle = !ledToggle));
      last = millis();
    }

}

void Loop::runSerial(){
    // Load a byte from the serial port and possibly process
    // add a completed message to the queue. 
   
    sdp.update(); 
}

/* Update the steppers step and direction 
*/
void Loop::feedSteppers(){

  if (!enabled){
    return;
  }

  if (finished_phase){
    stop();
    finishedPhase();
    finished_phase = false;
  }

  if (finished_phase == false and !running and planner.getQueueSize() > 0){
    if(nextPhase()){
      start();
    }
  }
}


void Loop::finishedPhase(){

  for(int i = 0; i < config.n_axes; i++){
    current_state.positions[i] = steppers[i]->getPosition();
  }

  current_state.queue_length = planner.getQueueSize();
  current_state.queue_time = planner.getQueueTime();

  sdp.sendDone(planner.getCurrentPhase().seq, current_state);

  if(planner.isEmpty()){
    sdp.sendEmpty(planner.getCurrentPhase().seq, current_state);
    //disable();
  }  
}

bool Loop::nextPhase(){

  if(isEmpty()){
    return false;
  }

  const PhaseJoints&  pj = planner.getNextPhase();

  int active_axes = 0;

  for (int axis = 0; axis < getConfig().n_axes; axis++){
    const JointSubSegment &jss = pj.moves[axis];
    if(jss.x != 0){
      active_axes++;
      state[axis].setParams(jss.t, jss.v_0, jss.v_1, jss.x, config.interrupt_delay);
    } else {
      state[axis].setParams(0, 0, 0, 0, 0);
    }
  }

  return active_axes > 0;
}


// Start the timers, if there are segments available. 
void Loop::start(){ 
    running = true;
    setDirection();
    setTimer.begin( stepISR, config.interrupt_delay);
    
}

void Loop::stop(){ 
  running = false;
  setTimer.end();
}

void Loop::setDirection(){
 
  switch (config.n_axes){
      //case 8: steppers[7]->enable(state[7].getDirection());
      //case 7: steppers[6]->enable(state[6].getDirection());
      case 6: steppers[5]->setDirection(state[5].getDirection());
      case 5: steppers[4]->setDirection(state[4].getDirection()); 
      case 4: steppers[3]->setDirection(state[3].getDirection()); 
      case 3: steppers[2]->setDirection(state[2].getDirection()); 
      case 2: steppers[1]->setDirection(state[1].getDirection());
      case 1: steppers[0]->setDirection(state[0].getDirection()); 
  }
}

void Loop::enable(){
  enabled = true;
  switch (config.n_axes){
      //case 8: steppers[7]->enable(state[7].getDirection());
      //case 7: steppers[6]->enable(state[6].getDirection());
      case 6: steppers[5]->enable(state[5].getDirection());
      case 5: steppers[4]->enable(state[4].getDirection()); 
      case 4: steppers[3]->enable(state[3].getDirection()); 
      case 3: steppers[2]->enable(state[2].getDirection()); 
      case 2: steppers[1]->enable(state[1].getDirection());
      case 1: steppers[0]->enable(state[0].getDirection()); 
  }
}

void Loop::disable(){
  enabled = false;

  switch (config.n_axes){
      //case 8: steppers[7]->disable();
      //case 7: steppers[6]->disable();
      case 6: steppers[5]->disable();
      case 5: steppers[4]->disable();
      case 4: steppers[3]->disable();
      case 3: steppers[2]->disable();
      case 2: steppers[1]->disable();
      case 1: steppers[0]->disable();
  }
}

void Loop::setConfig(Config* config_, bool eeprom_write){

  // FIXME EEPROM writing was failing?
  if (false and eeprom_write){
    EEPROM.put(EEPROM_OFFSET, *config_);
  }
  
  config.interrupt_delay = config_->interrupt_delay;
  config.n_axes = config_->n_axes;
  config.enable_active = config_->enable_active;
  config.debug_print = config_->debug_print;
  config.debug_tick = config_->debug_tick;
  
  planner.setNJoints(config.n_axes);
}

// Configure a stepper for an axis. Creates a new StepperInterface object
// for the axis
void Loop::setAxisConfig(AxisConfig* as, bool eeprom_write){
  
  int pos = 0;

  if(as->axis < config.n_axes){

    ser_printf("Config axis %d out of %d",as->axis+1, config.n_axes);
 
    // FIXME EEPROM writing was failing?
    if (false and eeprom_write)
      EEPROM.put(EEPROM_OFFSET+sizeof(Config)+(sizeof(AxisConfig)*as->axis), *as);

    // Clear out any old stepper instance
    if (steppers[as->axis] != 0)
      pos = steppers[as->axis]->getPosition(); // But save the position
      delete steppers[as->axis];

    // Then make a new one. 
    steppers[as->axis] = new StepInterface(as->axis, as->step_pin, as->direction_pin, as->enable_pin);

    steppers[as->axis]->setPosition(pos);

    steppers[as->axis]->SetEnableActive(config.enable_active);

    Joint joint(as->axis,static_cast< float >(as->v_max), static_cast< float >(as->a_max));
    planner.setJoint(joint);
  }
}

// Turn a move command into a move and add it to the planner
void Loop::processMove(const uint8_t* buffer_, size_t size){

    PacketHeader *ph = (PacketHeader*)buffer_;
    Moves *m = (Moves*)(buffer_ + sizeof(PacketHeader));
  
    Move move(getConfig().n_axes, ph->seq, m->segment_time, 0);

    switch(ph->code){
        case CommandCode::RMOVE:
     
        move.move_type = Move::MoveType::relative;
        break;
        
        case CommandCode::AMOVE:
        move.move_type = Move::MoveType::absolute;
        break;
        
        case CommandCode::JMOVE:
        move.move_type = Move::MoveType::jog;
        break;
        
        default: ; 
    }

    for (int axis = 0; axis < getConfig().n_axes; axis++){
      move.x[axis] = m->x[axis];
      // FIXME! This position update will only work for relative moves
      current_state.planner_positions[axis] += m->x[axis];
    }
    
    current_state.queue_length = planner.getQueueSize();
    current_state.queue_time = planner.getQueueTime();

    planner.push(move);
}

void Loop::printInfo(){

  sdp.printf("===========\n"
            "Queue Size : %d\r\n"
            "Queue Time : %d\r\n"
            "Running    : %d\r\n"
            "Enabled    : %d\r\n"
            "N Axes     : %d\r\n"
            "Joints     : %d\r\n"
            "Intr Delay : %d\r\n"
            "En Active  : %d\r\n"
            "Debug print: %d\r\n"
            "Debug tick : %d\r\n",
           planner.getQueueSize(), planner.getQueueTime(), running, enabled,
           config.n_axes,planner.getJoints().size(), config.interrupt_delay, 
           config.enable_active, config.debug_print, config.debug_tick) ;
  
  for(const Joint &j : planner.getJoints() ){

    if(j.n>=config.n_axes){
      continue;
    }

    StepInterface &stepper = getStepper(j.n);

    sdp.printf("-- Axis %d \r\n"
            "Step Pin   : %d\r\n"
            "Dir Pin    : %d\r\n"
            "Enable Pin : %d\r\n"
            "A Max      : %d\r\n"
            "V Max      : %d\r\n"
            "Position   : %d\r\n",
            j.n, stepper.stepPin, stepper.directionPin, stepper.enablePin, 
            static_cast<int>(j.a_max), static_cast<int>(j.v_max),
            stepper.getPosition()); 

  }

  for(const Segment *s : planner.getSegments()){
      stringstream ss;
      ss << *s << '\0';
      sdp.sendMessage(ss.str().c_str());
  }

}
