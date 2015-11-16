MyoSimulator
============

A library for simulating a Myo
------------------------------

This library provides the `MyoSim::Hub` as a drop in replacement for
`myo::Hub`. You can simulate any Myo event (e.g. `onPose` or `onGyroscopeData`)
by calling the corresponding function in `MyoSim::Hub`. The simulated hub will
call the corresponding functions in all of it's listeners in the order the
listeners were added.

Record and playback
-------------------

You can record events from the Myo into boost serializable structs using
`EventRecorder`. The events can then be played back using `EventPlayer`.

Interactive mode
----------------

`Hub::run` and `Hub::runOnce` are used to interactively simulate poses. The
simulated hub will wait for pose input via stdin and then simulate the
corresponding pose.

Known issues
------------

- Myo objects cannot be created without a physical Myo. Therefore events are
  simulated with a null Myo pointer unless a physical Myo is present. For event
  playback, the same number of physical Myos must be present as there are in the
  event session.

Dependecies
-----------

- [Boost](http://www.boost.org/)(optional - only needed if serializing events)
