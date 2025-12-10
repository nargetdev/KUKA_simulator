# ;;imperative;; manifest the kuka simulator in `./kuka_simulator/**`
The KUKA simulator "pretends" to be the real KUKA system and responds identically from the view of the network.  It receives binary buffers of gcode commands and enques commands into the EKI interface.  In the real system these goto commands are dequeued as the motions are completed on the robot.  This simulator will need to maintain a virtual position and upon execution of each individual gcode command should move the internal representation of a virtual tool in virtual space according to the feedrate.



# Description of the real system to emulate:

### note - Find in the `./REFERENCE` folder:
The gcode server implemented in KRL which runs on the KUKA windows machine (queue grotesque wretching sound from developers all around the world).
The gcode client in this case runs on a more sane (i.e. Linux based) ROS2 environment stuffing and sending the 48 byte binary buffer.


## component 1: the main loop

`./REFERENCES/kuka_gcode_server/_1_MAIN_gcodeToMotion.src`
`./REFERENCES/kuka_gcode_server/gcode.src`

receive the the gcode cmd on the binary EKI buffer


```REFERENCES/kuka_gcode_server/gcode.src
    ; get the binary representation
    eki_ret=EKI_GetString(EKI_SLOT[], "cmd", g_MAIN__gcode_raw_buffer[])

    ; place the fields into our gcode structure
    ZERO = 0
    CAST_FROM(g_MAIN__gcode_raw_buffer[], ZERO, received_gcode_cmd.seq, received_gcode_cmd.cmd_type, received_gcode_cmd.X, received_gcode_cmd.Y, received_gcode_cmd.Z, received_gcode_cmd.A, received_gcode_cmd.B, received_gcode_cmd.C, received_gcode_cmd.F, received_gcode_cmd.E)
    CAST_FROM(g_MAIN__gcode_raw_buffer[], ZERO, received_gcode_cmd.Bitmask)
```

our mock implementation here should do the same thing.


the queue fill will need to be sent back on the wire...
```REFERENCES/kuka_gcode_server/_1_MAIN_gcodeToMotion.src
      CAST_TO(g_MAIN__feedback_buffer[], ZERO, g_seq, g_queue_size)
      ; Fill remaining bytes (40 bytes / 10 INTs) with -27
      CAST_TO(g_MAIN__feedback_buffer[], ZERO, -27, -27, -27, -27, -27, -27, -27, -27, -27, -27)

      ; Only send if connection is alive ($FLAG[MAIN_FLAG])
      IF $FLAG[MAIN_FLAG] THEN
         ; Send the prepared buffer (only first 8 bytes contain valid data)
         eki_ret = EKI_Send(EKI_XML.MAIN[], g_MAIN__feedback_buffer[])
      ENDIF
```


## telemetry feedback ( 2__EX2__telemetryTCP.xml analogue )

```REFERENCES/kuka_gcode_server/_2_telemetry.sub
        if $flag[EX2_FLAG] then  ; If connection alive
            ZERO=0

            ; 0 .. 23
            CAST_TO(g_EX2_telemetry_buffer[],ZERO, $pos_act_mes.x, $pos_act_mes.y, $pos_act_mes.z, $pos_act_mes.a, $pos_act_mes.b, $pos_act_mes.c)
            ; 24 .. 47
            CAST_TO(g_EX2_telemetry_buffer[],ZERO, $axis_act_meas.a1, $axis_act_meas.a2, $axis_act_meas.a3, $axis_act_meas.a4, $axis_act_meas.a5, $axis_act_meas.a6)

            ; 48 .. 63
            CAST_TO(g_EX2_telemetry_buffer[],ZERO, g_seq, seq_in_flight, g_queue_size, FLAGS_WORD)

            ; 64 .. 95
            CAST_TO(g_EX2_telemetry_buffer[],ZERO, $VEL_ACT, $ACT_TOOL, $ACT_BASE, 76, 80, 84, 88, 92)

            ; 96 .. 119
            CAST_TO(g_EX2_telemetry_buffer[],ZERO, 96, 100, 104, 108, 112, 116, 120, 124)

            eki_ret=EKI_Send(EKI_XML.EX2[],g_EX2_telemetry_buffer[])
        endif
```

also reference: `REFERENCES/kuka_gcode_server/EKI/2__EX2__telemetryTCP.xml`



# TDD :: Validation

## 1. Single GCODE command.

**Initial STATE:** start the tool position at (0,0,0)

receive a gcode CMD:

```bachelor_OG.gcode
G1 X200 F100
```

;;what;; this is a very simple gcode statement commanding to position (200mm, 0mm, 0mm) at 100mm/s

;;expected behavior;; the simulator telemetry feed should reflect a motion (0,0,0) --> (200,0,0) over the course of 2 seconds.


