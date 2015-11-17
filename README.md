MyoSimulator
============

A library for simulating a Myo
------------------------------

This library provides the `myosim::Hub` as a drop in replacement for
`myo::Hub`. You can simulate any Myo event (e.g. `onPose` or `onGyroscopeData`)
by calling the corresponding function in `myosim::Hub`. The simulated hub will
call the corresponding functions in all of it's listeners in the order the
listeners were added.

Why?
----

Testing! Write better tests for Myo applications.

Record and playback
-------------------

You can record events from the Myo into boost serializable structs using
`EventRecorder`. The events can then be played back using `EventPlayerHub`. See
[Playback.cpp](examples/Playback.cpp) for a basic example and
[Serialize.cpp](examples/Serialize.cpp) for an example using boost serialization.

Interactive mode
----------------

[Interactive.cpp](examples/Interactive.cpp) is an example of using `myosim::Hub`
interactively to wait for pose input via stdin and then simulating the
corresponding pose.

Known issues
------------

- Myo objects cannot be created without a physical Myo. Therefore events are
  simulated with non-null Myo pointers only if the same number of myos are
  present as were used in the EventQueue. Otherwise all events are simulated
  with a null Myo pointer.

Dependecies
-----------

- [Boost](http://www.boost.org/) (optional - only needed if serializing events)
