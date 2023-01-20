PWM sample is based on AVR Pin Change Interrupt feature.
principle :
    - Each time, on a enabled (unmasked) port, the input signal change from Low to High or High to Low, a global interrupt is fired.
    - an attached ISR perform as follow :
        - get current timestamp (using eRCaGuy_Timer2_Counter lib wich provide better resolution than Arduino function : 0.5us instead of 4us) 
        - walkthrough all configured inputs to detect which one has changed by comparing previous state to current state :
            - if rising edge is detected : store current time for that input/channel
            - if falling edge is detected : compute pulse duration and mark that input/channel as ready for reading

Is there anything to improve here :

reminder :
the signal we want to acquire is a PWM signal with a pulse between 1000 us and 2000 us

Using a scope, we noticed that :
- ISR is triggered within 8-9us
- ISR takes arround 32us to compute all channels
- All channel rizing edge occurs at the same time (could depends on the receiver). Since time reference for all concerned channels is taken once at the beginning of the ISR, we are good here,
- Of course all falling edge dependens on actual controller position
    -> worst case, channel 1 falling edge occurs while processing others : we will need to wait for the next ISR to be fired to process it if we are alsready processing channel 2,3,.... Could lead to an increased measurment for that channel, by arround 30us : 3% compared to 1000us full scale signal.

Conclusiong :

- Having all PWN signal to acquire wired to the same AVR port is a good point since that will reduce number of ISR triggered, more particulary if all channels rizing edge occurs the same time.
- measure precision is about 3% worst case if multiple falling edge are closed together
- I don't see way to improve here
