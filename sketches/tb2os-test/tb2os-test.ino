#include <TaskScheduler.h>

// Create a scheduler object
Scheduler runner;

// Define task functions
void task1Callback();
void task2Callback();

// Define task objects
Task task1(1000, TASK_FOREVER, &task1Callback);
Task task2(2000, TASK_FOREVER, &task2Callback);

// States for state machine
enum State {IDLE, RUNNING, FINISHED};
State currentState = IDLE;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Starting tb2os...");

  // Add tasks to the scheduler
  runner.addTask(task1);
  runner.addTask(task2);

  // Enable tasks
  task1.enable();
  task2.enable();

  // Set initial state
  currentState = IDLE;
}

void loop() {
  // Run the scheduler
  runner.execute();

  // State machine logic
  switch (currentState) {
    case IDLE:
      Serial.println("State: IDLE");
      // Transition to RUNNING state
      currentState = RUNNING;
      break;
      
    case RUNNING:
      Serial.println("State: RUNNING");
      // Perform actions while running
      // Transition to FINISHED state after some condition
      // For demonstration, we'll use a timeout (not recommended for real use cases)
      delay(5000); // Simulate some running condition
      currentState = FINISHED;
      break;

    case FINISHED:
      Serial.println("State: FINISHED");
      // Perform cleanup or other actions
      // Transition back to IDLE state
      currentState = IDLE;
      break;
  }
}

// Task 1 callback function
void task1Callback() {
  Serial.println("Task 1 executed");
}

// Task 2 callback function
void task2Callback() {
  Serial.println("Task 2 executed");
}
