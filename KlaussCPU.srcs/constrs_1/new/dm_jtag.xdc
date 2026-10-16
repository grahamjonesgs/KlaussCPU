# On-chip debugger (DEBUG_EN=1): the BSCANE2 USER3 TCK clock of dm_jtag.
# Host JTAG runs at <= 30 MHz.  TCK is asynchronous to every core clock; the
# DMI crossing in dm_dtm uses toggle synchronizers (ASYNC_REG) with stable
# data registers, so all paths between the domains are false paths.
create_clock -period 33.333 -name dm_tck [get_pins -hierarchical -filter {NAME =~ *dm_jtag_i/bscan_i/TCK}]
set_clock_groups -asynchronous -group [get_clocks dm_tck]
