#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Safety watchdog and E-STOP manager.
 *
 * - Tracks the timestamp of the last valid command.
 * - If the command timeout expires, triggers safe-stop (brake + center steering).
 * - E-STOP latches until explicitly cleared.
 * - Heartbeat LED: 1 Hz normal, 5 Hz in E-STOP.
 */

/** Initialise safety subsystem (LED, timers). */
void safety_init(void);

/** Call every control cycle to update watchdog and LED. */
void safety_update(void);

/** Mark that a valid command was received (resets watchdog timer). */
void safety_feed_watchdog(void);

/** Activate E-STOP (latching). */
void safety_estop_activate(void);

/** Clear E-STOP (returns to normal). */
void safety_estop_clear(void);

/** Is E-STOP currently active? */
bool safety_is_estopped(void);

/** Has the command timeout expired? */
bool safety_is_timed_out(void);

/** Is the system in a safe-to-run state (no estop, no timeout)? */
bool safety_is_ok(void);

#endif /* SAFETY_H */
