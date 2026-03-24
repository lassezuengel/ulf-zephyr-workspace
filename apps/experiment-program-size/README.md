## Experiment: Program Size

Consider the following program, which consists of a single reactor that outputs an integer:

<p align="center">
	<img src="src/Fed_n4.svg" alt="Federation with star topology" title="Program with physical connection" width="50%" />
</p>

The RAM usage increases proportionally to the number of federates connected to the specific reactor. Thus, for the `Sink` reactors, the RAM usage is quite low, while for the `Source` reactor (the central reactor), the RAM usage is quite high. The following chart shows the RAM usage of the `Source` reactor as a function of the number of federates connected to it:

<p align="center">
	<img src="ram_chart_nrf52833.svg" alt="Chart of ram usage compared to number of fedederates" title="Program with physical connection" width="700" />
</p>