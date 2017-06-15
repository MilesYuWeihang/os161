#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <array.h>
/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */


//global variables
 typedef struct Car
{
  Direction origin;
  Direction destination;
} Car;
//static struct cv *intersectionCV;
static struct cv *NS;
static struct cv *NW;
static struct cv *NE;
static struct cv *SN;
static struct cv *SW;
static struct cv *SE;
static struct cv *WE;
static struct cv *WS;
static struct cv *WN;
static struct cv *EW;
static struct cv *EN;
static struct cv *ES;
static struct lock *trafficLock;
static struct array* mutexList;
static struct Car* temp;


//helper functions
bool isRightTurn(Car *);
bool canGoTogether(Car *, Car *);
bool tryEnter(Car *);
bool leaveMutex(Direction, Direction);
struct cv *chooseCV(Direction, Direction);
//determine if this car is turning rigt
bool isRightTurn(Car *c){
  if((c->origin == north)&&(c->destination == west)){
    return true;
  }
  if((c->origin == south)&&(c->destination == east)){ 
    return true;
  }
  if((c->origin == west)&&(c->destination == south)){
    return true;
  }
  if((c->origin == east)&&(c->destination == north)){
    return true;
  }
  return false;
}

//determine can car a and b can go together in the mutex area
bool canGoTogether(Car *a, Car *b){
  //same origin
  if (a->origin == b->origin){
    return true;
  }
  //same line
  if ((a->origin == b->destination)&&(a->destination == b->origin)){
    return true;
  }
  //right turn
  if((a->destination != b->destination)&&(isRightTurn(a) || isRightTurn(b))){
    return true;
  }
  return false;
}

//determine if this car can gotogether with all cars in mutex
bool tryEnter(Car *c){
  unsigned a = array_num(mutexList);
  //kprintf("list here: \n");
    //kprintf("%d, %d \n", c->origin, c->destination);
  for (unsigned i = 0; i< a ; i++){
    
    temp = array_get(mutexList, i);
    //kprintf("origin:%d, destination:%d \n", temp->origin,temp->destination);
    if(!canGoTogether(c, temp)){
      //kprintf("this car wait\n");
      cv_wait(chooseCV(temp->origin, temp->destination), trafficLock);
      return false;
    }
  }
  
  //kprintf("this car entered\n");
  return true;

}

// move car from mutexList
bool leaveMutex(Direction origin, Direction destination){
  unsigned a = array_num(mutexList);
  for (unsigned i = 0; i< a ; i++){
    temp = array_get(mutexList, i);
    if((temp->origin == origin)&& (temp->destination == destination)){
      array_remove(mutexList,i);
      return true;
    }
  }
  return false;
}

struct cv *chooseCV(Direction d, Direction e){
  switch (d){
    
    case north:   switch(e){
        case south: return NS;
        case west: return NW;
        case east: return NE;
        default: break;
      };
    
    case south:
      switch(e){
        case north: return SN;
        case west: return SW;
        case east: return SE;
        default:  break;
      };
    case west: 
      switch(e){
        case south: return WS;
        case north: return WN;
        case east: return WE;
        default:  break;
      };
    case east: 
      switch(e){
        case south: return ES;
        case west: return EW;
        case north: return EN;
        default:  break;
      };
  }
  return NS;
}

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */

  /*intersectionCV = cv_create("intersectionCV");
  if (intersectionCV == NULL) {
    panic("could not create intersection CV");
  }
  */
  trafficLock = lock_create("trafficLock");
  if (trafficLock == NULL) {
    panic("could not create lock");
  }
  NS = cv_create("NS");
  NW = cv_create("NW");
  NE = cv_create("NE");
  EN = cv_create("EN");
  ES = cv_create("ES");
  EW = cv_create("EW");
  SW = cv_create("SW");
  SN = cv_create("SN");
  SE = cv_create("SE");
  WN = cv_create("WN");
  WE = cv_create("WE");
  WS = cv_create("WS");
  
  mutexList = array_create();
  array_init(mutexList);



  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  //KASSERT(intersectionCV != NULL);
  //cv_destroy(intersectionCV);
  cv_destroy(NS);
  cv_destroy(NW);
  cv_destroy(NE);
  cv_destroy(EN);
  cv_destroy(ES);
  cv_destroy(EW);
  cv_destroy(SN);
  cv_destroy(SW);
  cv_destroy(SE);
  cv_destroy(WN);
  cv_destroy(WE);
  cv_destroy(WS);
  KASSERT(trafficLock != NULL);
  lock_destroy(trafficLock);
  array_destroy(mutexList);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  lock_acquire(trafficLock);
  Car* tmp = kmalloc(sizeof(struct Car));
  tmp->origin = origin;
  tmp->destination = destination;
  while(1){
    //if this car cannot enter, it will keep loop until get enter
    //wc_wait will release and reaquire the trafficLock
    if(tryEnter(tmp)){
      array_add(mutexList, tmp,NULL);
      break;
    }
 }
  lock_release(trafficLock);

}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  lock_acquire(trafficLock);
   //kprintf("\n");
   // kprintf("%d, %d \n", origin, destination);
  if(leaveMutex(origin,destination)){
      cv_broadcast(chooseCV(origin,destination),trafficLock);
   // kprintf("leave safe \n"); 
  }
  else{
    panic("this car is not in mutex");
  }
  lock_release(trafficLock);

}
