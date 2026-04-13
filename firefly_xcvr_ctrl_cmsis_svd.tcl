package require Tcl 8.5

set script_dir [file dirname [info script]]
set helper_file [file normalize [file join $script_dir .. dashboard_infra cmsis_svd lib mu3e_cmsis_svd.tcl]]
source $helper_file

namespace eval ::mu3e::cmsis::spec {}

proc ::mu3e::cmsis::spec::raw_register {name offset description access} {
    return [::mu3e::cmsis::svd::register $name $offset \
        -description $description \
        -access $access \
        -fields [list \
            [::mu3e::cmsis::svd::field value 0 32 \
                -description {Raw 32-bit register word. Refer to the register description for read-versus-write semantics.} \
                -access $access]]]
}

proc ::mu3e::cmsis::spec::build_device {} {
    set registers [list \
        [::mu3e::cmsis::spec::raw_register FF1_TEMP_STATUS 0x00 {Firefly 1 status/control word. Reads return temperature in bits [7:0]. When hidden CSR mode is enabled, bits [27:24] expose present_n and int_n pins. Writing bit 0 enables or disables continuous I2C polling.} read-write] \
        [::mu3e::cmsis::spec::raw_register FF1_VCC_RESET 0x04 {Firefly 1 VCC/control word. Reads return the low 16 bits of the VCC measurement. Writing bit 0 requests a Firefly reset pulse.} read-write] \
        [::mu3e::cmsis::spec::raw_register FF1_RXPWR1_HIDDEN 0x08 {Firefly 1 RX power channel 1 on read. Writing bit 0 enables or disables the hidden CSR page that augments FF1_TEMP_STATUS.} read-write] \
        [::mu3e::cmsis::spec::raw_register FF1_RXPWR2 0x0C {Firefly 1 received optical power for channel 2 in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF1_RXPWR3 0x10 {Firefly 1 received optical power for channel 3 in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF1_RXPWR4 0x14 {Firefly 1 received optical power for channel 4 in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF1_ALARM 0x18 {Firefly 1 latched alarm/status flags. Bits clear on read inside the module according to the Firefly alarm semantics.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF2_TEMP_STATUS 0x1C {Firefly 2 status word. Reads return temperature in bits [7:0].} read-only] \
        [::mu3e::cmsis::spec::raw_register FF2_VCC 0x20 {Firefly 2 VCC measurement in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF2_RXPWR1 0x24 {Firefly 2 received optical power for channel 1 in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF2_RXPWR2 0x28 {Firefly 2 received optical power for channel 2 in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF2_RXPWR3 0x2C {Firefly 2 received optical power for channel 3 in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF2_RXPWR4 0x30 {Firefly 2 received optical power for channel 4 in the low 16 bits.} read-only] \
        [::mu3e::cmsis::spec::raw_register FF2_ALARM 0x34 {Firefly 2 latched alarm/status flags. Bits clear on read inside the module according to the Firefly alarm semantics.} read-only]]

    return [::mu3e::cmsis::svd::device MU3E_FIREFLY_XCVR_CTRL \
        -version 26.0.330 \
        -description {CMSIS-SVD description of the Firefly optical-module I2C monitor aperture. BaseAddress is 0 because this file describes the relative CSR aperture of the IP; system integration supplies the live slave base address.} \
        -peripherals [list \
            [::mu3e::cmsis::svd::peripheral FIREFLY_XCVR_CTRL 0x0 \
                -description {Relative 14-word Firefly monitor aperture. Read semantics dominate; the first three words also accept control writes described in each register description.} \
                -groupName MU3E_FIREFLY \
                -addressBlockSize 0x38 \
                -registers $registers]]]
}

if {[info exists ::argv0] &&
    [file normalize $::argv0] eq [file normalize [info script]]} {
    set out_path [file join $script_dir firefly_xcvr_ctrl.svd]
    if {[llength $::argv] >= 1} {
        set out_path [lindex $::argv 0]
    }
    ::mu3e::cmsis::svd::write_device_file \
        [::mu3e::cmsis::spec::build_device] $out_path
}
