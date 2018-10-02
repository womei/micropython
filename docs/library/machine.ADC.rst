.. currentmodule:: machine
.. _machine.ADC:

class ADC -- analog to digital conversion
=========================================

The ADC class provides an interface to analog-to-digital convertors, and
represents a single endpoint that can sample a continuous voltage and
convert it to a discretised value.

For extra control over ADC sampling see :ref:`machine.ADCBlock <machine.ADCBlock>`.

Example usage::

   import machine

   adc = machine.ADC(pin)   # create an ADC object acting on a pin
   val = adc.read_v()       # read an analog value in Volts
   val = adc.read_u16()     # read a raw analog value in the range 0-65535

Constructors
------------

.. class:: ADC(id, \*, sample_ns, gain_mult, gain_div)

   Access the ADC associated with a source identified by *id*.  This
   *id* may be an integer (usually specifying a channel number), a
   :ref:`Pin <machine.Pin>` object, or other value supported by the
   underlying machine.

   If additional keyword-arguments are given then they will configure
   various aspects of the ADC.  If not given, these settings will take
   previous or default values.  The settings are:

     - *sample_ns* is the sampling time in nanoseconds.

     - *gain_mult* and *gain_div* specify the total gain applied to the
       input before conversion, via the formula ``gain = gain_mult / gain_div``.
       If only one of these arguments is specified then the other one is
       taken as equal to 1.

Methods
-------

.. method:: ADC.init(sample_ns, gain_mult, gain_div)

   Apply the given settings to the ADC.  Only those arguments that are
   specified will be changed.  See the ADC constructor above for what the
   arguments are.

.. method:: ADC.read_u16()

   Take an analog reading and return an integer in the range 0-65535.
   The return value represents the raw reading taken by the ADC, scaled
   such that the minimum value is 0 and the maximum value is 65535.

.. method:: ADC.read_v()

   Take an analog reading and return a floating-point value with units of
   Volts.  It is up to the particular port whether or not this value is
   calibrated, and how calibration is done.

.. method:: ADC.read_mv()

   Take an analog reading and return an integer value with units of
   milli-Volts.  It is up to the particular port whether or not this value
   is calibrated, and how calibration is done.
