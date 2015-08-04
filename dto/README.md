
Black Box Overlay
=================

This overlay is for the Black Box cape which enables functionality
for robotic research capabilities.


<table>
  <tr>
    <td colspan="8" align="center"> <b> PRU INPUTS (RADIO) [10CH] </b> </td>
  </tr>
  <tr>
    <th align="center"> Header       </th>
    <th align="center"> $PINS        </th>
    <th align="center"> Address      </th>
    <th align="center"> Offset       </th>
    <th align="center"> Name         </th>
    <th align="center"> GPIO         </th>
    <th align="center"> Mode         </th>
    <th align="left"  > Description  </th>
  </tr>
  <tr>
    <td align="center"> P9_31       </td>
    <td align="center"> 100         </td>
    <td align="center"> 990         </td>
    <td align="center"> 190         </td>
    <td align="center"> PRU0_r31_00 </td>
    <td align="center"> gpio3[14]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P9_29       </td>
    <td align="center"> 101         </td>
    <td align="center"> 994         </td>
    <td align="center"> 194         </td>
    <td align="center"> PRU0_r31_01 </td>
    <td align="center"> gpio3[15]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P9_30       </td>
    <td align="center"> 102         </td>
    <td align="center"> 998         </td>
    <td align="center"> 198         </td>
    <td align="center"> PRU0_r31_02 </td>
    <td align="center"> gpio3[16]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P9_28       </td>
    <td align="center"> 103         </td>
    <td align="center"> 99C         </td>
    <td align="center"> 19C         </td>
    <td align="center"> PRU0_r31_03 </td>
    <td align="center"> gpio3[17]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P9_42       </td>
    <td align="center"> 89          </td>
    <td align="center"> 9A0         </td>
    <td align="center"> 1A0         </td>
    <td align="center"> PRU0_r31_04 </td>
    <td align="center"> gpio3[18]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P9_27       </td>
    <td align="center"> 105         </td>
    <td align="center"> 9A4         </td>
    <td align="center"> 1A4         </td>
    <td align="center"> PRU0_r31_05 </td>
    <td align="center"> gpio3[19]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P9_41       </td>
    <td align="center"> 109         </td>
    <td align="center"> 9A8         </td>
    <td align="center"> 1A8         </td>
    <td align="center"> PRU0_r31_06 </td>
    <td align="center"> gpio3[20]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P9_25       </td>
    <td align="center"> 107         </td>
    <td align="center"> 9AC         </td>
    <td align="center"> 1AC         </td>
    <td align="center"> PRU0_r31_07 </td>
    <td align="center"> gpio3[21]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P8_16       </td>
    <td align="center"> 14          </td>
    <td align="center"> 838         </td>
    <td align="center"> 038         </td>
    <td align="center"> PRU0_r31_14 </td>
    <td align="center"> gpio1[14]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
  <tr>
    <td align="center"> P8_15       </td>
    <td align="center"> 15          </td>
    <td align="center"> 83C         </td>
    <td align="center"> 03C         </td>
    <td align="center"> PRU0_r31_15 </td>
    <td align="center"> gpio1[15]   </td>
    <td align="center"> 6           </td>
    <td align="left"  > INPUT_X     </td>
  </tr>
</table>



<!--
<table>
  <tr>
    <td colspan="8" align="center"> <b> PRU OUTPUTS (ESC/SERVO) [12CH] </b> </td>
  </tr>
  <tr>
    <th align="center"> Header       </th>
    <th align="center"> $PINS        </th>
    <th align="center"> Address      </th>
    <th align="center"> Offset       </th>
    <th align="center"> Name         </th>
    <th align="center"> GPIO         </th>
    <th align="center"> Mode         </th>
    <th align="left"  > Description  </th>
  </tr>
  <tr>
    <td align="center"> P8_45       </td>
    <td align="center"> 40          </td>
    <td align="center"> 8A0         </td>
    <td align="center"> 0A0         </td>
    <td align="center"> PRU1_r30_00 </td>
    <td align="center"> gpio2[06]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_46       </td>
    <td align="center"> 41          </td>
    <td align="center"> 8A4         </td>
    <td align="center"> 0A4         </td>
    <td align="center"> PRU1_r30_01 </td>
    <td align="center"> gpio2[07]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_43       </td>
    <td align="center"> 42          </td>
    <td align="center"> 8A8         </td>
    <td align="center"> 0A8         </td>
    <td align="center"> PRU1_r30_02 </td>
    <td align="center"> gpio2[08]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_44       </td>
    <td align="center"> 43          </td>
    <td align="center"> 8AC         </td>
    <td align="center"> 0AC         </td>
    <td align="center"> PRU1_r30_03 </td>
    <td align="center"> gpio2[09]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_41       </td>
    <td align="center"> 44          </td>
    <td align="center"> 8B0         </td>
    <td align="center"> 0B0         </td>
    <td align="center"> PRU1_r30_04 </td>
    <td align="center"> gpio2[10]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_42       </td>
    <td align="center"> 45          </td>
    <td align="center"> 8B4         </td>
    <td align="center"> 0B4         </td>
    <td align="center"> PRU1_r30_05 </td>
    <td align="center"> gpio2[11]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_39       </td>
    <td align="center"> 46          </td>
    <td align="center"> 8B8         </td>
    <td align="center"> 0B8         </td>
    <td align="center"> PRU1_r30_06 </td>
    <td align="center"> gpio2[12]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_40       </td>
    <td align="center"> 47          </td>
    <td align="center"> 8BC         </td>
    <td align="center"> 0BC         </td>
    <td align="center"> PRU1_r30_07 </td>
    <td align="center"> gpio2[13]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_27       </td>
    <td align="center"> 56          </td>
    <td align="center"> 8E0         </td>
    <td align="center"> 0E0         </td>
    <td align="center"> PRU1_r30_08 </td>
    <td align="center"> gpio2[22]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_28       </td>
    <td align="center"> 58          </td>
    <td align="center"> 8E8         </td>
    <td align="center"> 0E8         </td>
    <td align="center"> PRU1_r30_10 </td>
    <td align="center"> gpio2[24]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_21       </td>
    <td align="center"> 32          </td>
    <td align="center"> 880         </td>
    <td align="center"> 080         </td>
    <td align="center"> PRU1_r30_12 </td>
    <td align="center"> gpio1[30]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
  <tr>
    <td align="center"> P8_20       </td>
    <td align="center"> 33          </td>
    <td align="center"> 884         </td>
    <td align="center"> 084         </td>
    <td align="center"> PRU1_r30_13 </td>
    <td align="center"> gpio1[31]   </td>
    <td align="center"> 5           </td>
    <td align="left"  > OUTPUT_X    </td>
  </tr>
</table>
-->



<!--
<table>
  <tr>
    <td colspan="8" align="center"> <b> UART DEVICES [4CH] </b> </td>
  </tr>
  <tr>
    <th align="center"> Header       </th>
    <th align="center"> $PINS        </th>
    <th align="center"> Address      </th>
    <th align="center"> Offset       </th>
    <th align="center"> Name         </th>
    <th align="center"> GPIO         </th>
    <th align="center"> Mode         </th>
    <th align="left"  > Description  </th>
  </tr>
  <tr>
    <td align="center"> P9_24     </td>
    <td align="center"> 97        </td>
    <td align="center"> 984       </td>
    <td align="center"> 184       </td>
    <td align="center"> UART1_TX  </td>
    <td align="center"> gpio0[15] </td>
    <td align="center"> 0         </td>
    <td align="left"  > UART1_TX  </td>
  </tr>
  <tr>
    <td align="center"> P9_26     </td>
    <td align="center"> 96        </td>
    <td align="center"> 980       </td>
    <td align="center"> 180       </td>
    <td align="center"> UART1_RX  </td>
    <td align="center"> gpio0[14] </td>
    <td align="center"> 0         </td>
    <td align="left"  > UART1_RX  </td>
  </tr>
  <tr>
    <td align="center"> P9_21     </td>
    <td align="center"> 85        </td>
    <td align="center"> 954       </td>
    <td align="center"> 154       </td>
    <td align="center"> UART2_TX  </td>
    <td align="center"> gpio0[03] </td>
    <td align="center"> 1         </td>
    <td align="left"  > UART2_TX  </td>
  </tr>
  <tr>
    <td align="center"> P9_22     </td>
    <td align="center"> 84        </td>
    <td align="center"> 950       </td>
    <td align="center"> 150       </td>
    <td align="center"> UART2_RX  </td>
    <td align="center"> gpio0[02] </td>
    <td align="center"> 1         </td>
    <td align="left"  > UART2_RX  </td>
  </tr>
  <tr>
    <td align="center"> P9_13     </td>
    <td align="center"> 29        </td>
    <td align="center"> 874       </td>
    <td align="center"> 074       </td>
    <td align="center"> UART4_TX  </td>
    <td align="center"> gpio0[31] </td>
    <td align="center"> 6         </td>
    <td align="left"  > UART4_TX  </td>
  </tr>
  <tr>
    <td align="center"> P9_11     </td>
    <td align="center"> 28        </td>
    <td align="center"> 870       </td>
    <td align="center"> 070       </td>
    <td align="center"> UART4_RX  </td>
    <td align="center"> gpio0[30] </td>
    <td align="center"> 6         </td>
    <td align="left"  > UART4_RX  </td>
  </tr>
  <tr>
    <td align="center"> P8_37     </td>
    <td align="center"> 48        </td>
    <td align="center"> 8C0       </td>
    <td align="center"> 0C0       </td>
    <td align="center"> UART5_TX  </td>
    <td align="center"> gpio2[14] </td>
    <td align="center"> 4         </td>
    <td align="left"  > UART5_TX  </td>
  </tr>
  <tr>
    <td align="center"> P8_38     </td>
    <td align="center"> 49        </td>
    <td align="center"> 8C4       </td>
    <td align="center"> 0C4       </td>
    <td align="center"> UART5_RX  </td>
    <td align="center"> gpio2[15] </td>
    <td align="center"> 4         </td>
    <td align="left"  > UART5_RX  </td>
  </tr>
</table>
-->

<!--
<table>
  <tr>
    <td colspan="8" align="center"> <b> I2C DEVICES [2CH] </b> </td>
  </tr>
  <tr>
    <th align="center"> Header       </th>
    <th align="center"> $PINS        </th>
    <th align="center"> Address      </th>
    <th align="center"> Offset       </th>
    <th align="center"> Name         </th>
    <th align="center"> GPIO         </th>
    <th align="center"> Mode         </th>
    <th align="left"  > Description  </th>
  </tr>
  <tr>
    <td align="center"> P9_17        </td>
    <td align="center"> 87           </td>
    <td align="center"> 95C          </td>
    <td align="center"> 15C          </td>
    <td align="center"> I2C1_SCL     </td>
    <td align="center"> gpio0[05]    </td>
    <td align="center"> 2            </td>
    <td align="left"  >              </td>
  </tr>
  <tr>
    <td align="center"> P9_18        </td>
    <td align="center"> 86           </td>
    <td align="center"> 958          </td>
    <td align="center"> 158          </td>
    <td align="center"> I2C1_SDA     </td>
    <td align="center"> gpio0[04]    </td>
    <td align="center"> 2            </td>
    <td align="left"  >              </td>
  </tr>
  <tr>
    <td align="center"> P9_19        </td>
    <td align="center"> 95           </td>
    <td align="center"> 97C          </td>
    <td align="center"> 17C          </td>
    <td align="center"> I2C2_SCL     </td>
    <td align="center"> gpio0[13]    </td>
    <td align="center"> 3            </td>
    <td align="left"  >              </td>
  </tr>
  <tr>
    <td align="center"> P9_20        </td>
    <td align="center"> 94           </td>
    <td align="center"> 978          </td>
    <td align="center"> 178          </td>
    <td align="center"> I2C2_SDA     </td>
    <td align="center"> gpio0[12]    </td>
    <td align="center"> 3            </td>
    <td align="left"  >              </td>
  </tr>
</table>
-->

<!--
<table>
  <tr>
    <td colspan="8" align="center"> <b> STANDARD PWM [XCH] </b> </td>
  </tr>
  <tr>
    <th align="center"> Header       </th>
    <th align="center"> $PINS        </th>
    <th align="center"> Address      </th>
    <th align="center"> Offset       </th>
    <th align="center"> Name         </th>
    <th align="center"> GPIO         </th>
    <th align="center"> Mode         </th>
    <th align="left"  > Description  </th>
  </tr>
  <tr>
    <td align="center"> P9_14        </td>
    <td align="center"> 18           </td>
    <td align="center"> 848          </td>
    <td align="center"> 048          </td>
    <td align="center"> EHRPWM1A     </td>
    <td align="center"> gpio1[18]    </td>
    <td align="center"> 6            </td>
    <td align="left"  > PWM_X        </td>
  </tr>
  <tr>
    <td align="center"> P9_16        </td>
    <td align="center"> 19           </td>
    <td align="center"> 84C          </td>
    <td align="center"> 04C          </td>
    <td align="center"> EHRPWM1B     </td>
    <td align="center"> gpio1[19]    </td>
    <td align="center"> 6            </td>
    <td align="left"  > PWM_X        </td>
  </tr>
  <tr>
    <td align="center"> P8_13        </td>
    <td align="center"> 09           </td>
    <td align="center"> 824          </td>
    <td align="center"> 024          </td>
    <td align="center"> EHRPWM2B     </td>
    <td align="center"> gpio0[23]    </td>
    <td align="center"> 4            </td>
    <td align="left"  > PWM_X        </td>
  </tr>
  <tr>
    <td align="center"> P8_19        </td>
    <td align="center"> 08           </td>
    <td align="center"> 820          </td>
    <td align="center"> 020          </td>
    <td align="center"> EHRPWM2A     </td>
    <td align="center"> gpio0[22]    </td>
    <td align="center"> 4            </td>
    <td align="left"  > PWM_X        </td>
  </tr>
  <tr>
    <td align="center"> P8_34        </td>
    <td align="center"> 51           </td>
    <td align="center"> 8CC          </td>
    <td align="center"> 0CC          </td>
    <td align="center"> EHRPWM1B     </td>
    <td align="center"> gpio2[17]    </td>
    <td align="center"> 2            </td>
    <td align="left"  > PWM_X        </td>
  </tr>
  <tr>
    <td align="center"> P8_36        </td>
    <td align="center"> 50           </td>
    <td align="center"> 8C8          </td>
    <td align="center"> 0C8          </td>
    <td align="center"> EHRPWM1A     </td>
    <td align="center"> gpio2[16]    </td>
    <td align="center"> 2            </td>
    <td align="left"  > PWM_X        </td>
  </tr>
</table>
-->

<!--
<table>
  <tr>
    <td colspan="8" align="center"> <b> TIMERS [XCH] </b> </td>
  </tr>
  <tr>
    <th align="center"> Header       </th>
    <th align="center"> $PINS        </th>
    <th align="center"> Address      </th>
    <th align="center"> Offset       </th>
    <th align="center"> Name         </th>
    <th align="center"> GPIO         </th>
    <th align="center"> Mode         </th>
    <th align="left"  > Description  </th>
  </tr>
  <tr>
    <td align="center"> P8_07        </td>
    <td align="center"> 36           </td>
    <td align="center"> 890          </td>
    <td align="center"> 090          </td>
    <td align="center"> TIMER4       </td>
    <td align="center"> gpio2[02]    </td>
    <td align="center"> 2            </td>
    <td align="left"  > TIMER_X      </td>
  </tr>
  <tr>
    <td align="center"> P8_08        </td>
    <td align="center"> 37           </td>
    <td align="center"> 894          </td>
    <td align="center"> 094          </td>
    <td align="center"> TIMER7       </td>
    <td align="center"> gpio2[03]    </td>
    <td align="center"> 2            </td>
    <td align="left"  > TIMER_X      </td>
  </tr>
  <tr>
    <td align="center"> P8_09        </td>
    <td align="center"> 39           </td>
    <td align="center"> 89C          </td>
    <td align="center"> 09C          </td>
    <td align="center"> TIMER5       </td>
    <td align="center"> gpio2[05]    </td>
    <td align="center"> 2            </td>
    <td align="left"  > TIMER_X      </td>
  </tr>
  <tr>
    <td align="center"> P8_10        </td>
    <td align="center"> 38           </td>
    <td align="center"> 898          </td>
    <td align="center"> 098          </td>
    <td align="center"> TIMER6       </td>
    <td align="center"> gpio2[04]    </td>
    <td align="center"> 2            </td>
    <td align="left"  > TIMER_X      </td>
  </tr>
</table>
-->

<!--
<table>
  <tr>
    <td colspan="2" align="center"> <b> ANALOG TO DIGITAL [7CH] </b> </td>
  </tr>
  <tr>
    <th align="center"> Header </th>
    <th align="center"> Name   </th>
  </tr>
  <tr>
    <td align="center"> P9_34  </th>
    <td align="center"> AGND   </th>
  </tr>
  <tr>
    <td align="center"> P9_32  </th>
    <td align="center"> VADC   </th>
  </tr>
  <tr>
    <td align="center"> P9_39  </th>
    <td align="center"> AIN0   </th>
  </tr>
  <tr>
    <td align="center"> P9_40  </th>
    <td align="center"> AIN1   </th>
  </tr>
  <tr>
    <td align="center"> P9_37  </th>
    <td align="center"> AIN2   </th>
  </tr>
  <tr>
    <td align="center"> P9_38  </th>
    <td align="center"> AIN3   </th>
  </tr>
  <tr>
    <td align="center"> P9_33  </th>
    <td align="center"> AIN4   </th>
  </tr>
  <tr>
    <td align="center"> P9_36  </th>
    <td align="center"> AIN5   </th>
  </tr>
  <tr>
    <td align="center"> P9_35  </th>
    <td align="center"> AIN6   </th>
  </tr>
</table>
-->


<table>
  <tr>
    <td colspan="9" align="center"> <b> MASTER PIN ALLOCATION </b> </td>
  </tr>
  <tr>
    <td align="right"  > GND </td>
    <td align="right"  > P9_01 </td>
    <td align="left"   > P9_02 </td>
    <td align="left"   > GND </td>
    <td align="center" > </td>
    <td align="right"  > GND </td>
    <td align="right"  > P8_01 </td>
    <td align="left"   > P8_02 </td>
    <td align="left"   > GND </td>
  </tr>
  <tr>
    <td align="right"  > DC_3.3V </td>
    <td align="right"  > P9_03 </td>
    <td align="left"   > P9_04 </td>
    <td align="left"   > DC_3.3V </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_03 </td>
    <td align="left"   > P8_04 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > VDD_5V </td>
    <td align="right"  > P9_05 </td>
    <td align="left"   > P9_06 </td>
    <td align="left"   > VDD_5V </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_05 </td>
    <td align="left"   > P8_06 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > SYS_5V </td>
    <td align="right"  > P9_07 </td>
    <td align="left"   > P9_08 </td>
    <td align="left"   > SYS_5V </td>
    <td align="center" > </td>
    <td align="right"  > TIMER4 </td>
    <td align="right"  > P8_07 </td>
    <td align="left"   > P8_08 </td>
    <td align="left"   > TIMER7 </td>
  </tr>
  <tr>
    <td align="right"  > PWR_BUT </td>
    <td align="right"  > P9_09 </td>
    <td align="left"   > P9_10 </td>
    <td align="left"   > SYS_RESET </td>
    <td align="center" > </td>
    <td align="right"  > TIMER5 </td>
    <td align="right"  > P8_09 </td>
    <td align="left"   > P8_10 </td>
    <td align="left"   > TIMER6 </td>
  </tr>
  <tr>
    <td align="right"  > UART4_RX </td>
    <td align="right"  > P9_11 </td>
    <td align="left"   > P9_12 </td>
    <td align="left"   > N/A </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_11 </td>
    <td align="left"   > P8_12 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > UART4_TX </td>
    <td align="right"  > P9_13 </td>
    <td align="left"   > P9_14 </td>
    <td align="left"   > PWM_1A </td>
    <td align="center" > </td>
    <td align="right"  > PWM_2B </td>
    <td align="right"  > P8_13 </td>
    <td align="left"   > P8_14 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > N/A </td>
    <td align="right"  > P9_15 </td>
    <td align="left"   > P9_16 </td>
    <td align="left"   > PWM_1B </td>
    <td align="center" > </td>
    <td align="right"  > PRU0_r31_15 </td>
    <td align="right"  > P8_15 </td>
    <td align="left"   > P8_16 </td>
    <td align="left"   > PRU0_r31_14 </td>
  </tr>
  <tr>
    <td align="right"  > 12C1_SCL </td>
    <td align="right"  > P9_17 </td>
    <td align="left"   > P9_18 </td>
    <td align="left"   > I2C1_SDA </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_17 </td>
    <td align="left"   > P8_18 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > I2C2_SCL </td>
    <td align="right"  > P9_19 </td>
    <td align="left"   > P9_20 </td>
    <td align="left"   > I2C2_SDA </td>
    <td align="center" > </td>
    <td align="right"  > PWM_2A </td>
    <td align="right"  > P8_19 </td>
    <td align="left"   > P8_20 </td>
    <td align="left"   > PRU1_r30_13 </td>
  </tr>
  <tr>
    <td align="right"  > UART2_TX </td>
    <td align="right"  > P9_21 </td>
    <td align="left"   > P9_22 </td>
    <td align="left"   > UART2_RX </td>
    <td align="center" > </td>
    <td align="right"  > PRU1_r30_12 </td>
    <td align="right"  > P8_21 </td>
    <td align="left"   > P8_22 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > N/A </td>
    <td align="right"  > P9_23 </td>
    <td align="left"   > P9_24 </td>
    <td align="left"   > UART1_TX </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_23 </td>
    <td align="left"   > P8_24 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > PRU0_r31_07 </td>
    <td align="right"  > P9_25 </td>
    <td align="left"   > P9_26 </td>
    <td align="left"   > UART1_RX </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_25 </td>
    <td align="left"   > P8_26 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > PRU0_r31_05 </td>
    <td align="right"  > P9_27 </td>
    <td align="left"   > P9_28 </td>
    <td align="left"   > PRU0_r31_03 </td>
    <td align="center" > </td>
    <td align="right"  > PRU1_r30_08 </td>
    <td align="right"  > P8_27 </td>
    <td align="left"   > P8_28 </td>
    <td align="left"   > PRU1_r30_10 </td>
  </tr>
  <tr>
    <td align="right"  > PRU0_r31_01 </td>
    <td align="right"  > P9_29 </td>
    <td align="left"   > P9_30 </td>
    <td align="left"   > PRU0_r31_02 </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_29 </td>
    <td align="left"   > P8_30 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > PRU0_r31_00 </td>
    <td align="right"  > P9_31 </td>
    <td align="left"   > P9_32 </td>
    <td align="left"   > VADC </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_31 </td>
    <td align="left"   > P8_32 </td>
    <td align="left"   > N/A </td>
  </tr>
  <tr>
    <td align="right"  > AIN4 </td>
    <td align="right"  > P9_33 </td>
    <td align="left"   > P9_34 </td>
    <td align="left"   > AGND </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_33 </td>
    <td align="left"   > P8_34 </td>
    <td align="left"   > PWM_1B </td>
  </tr>
  <tr>
    <td align="right"  > AIN6 </td>
    <td align="right"  > P9_35 </td>
    <td align="left"   > P9_36 </td>
    <td align="left"   > AIN5 </td>
    <td align="center" > </td>
    <td align="right"  > N/A </td>
    <td align="right"  > P8_35 </td>
    <td align="left"   > P8_36 </td>
    <td align="left"   > PWM_1A </td>
  </tr>
  <tr>
    <td align="right"  > AIN2 </td>
    <td align="right"  > P9_37 </td>
    <td align="left"   > P9_38 </td>
    <td align="left"   > AIN3 </td>
    <td align="center" > </td>
    <td align="right"  > UART5_TX </td>
    <td align="right"  > P8_37 </td>
    <td align="left"   > P8_38 </td>
    <td align="left"   > UART5_RX </td>
  </tr>
  <tr>
    <td align="right"  > AIN0 </td>
    <td align="right"  > P9_39 </td>
    <td align="left"   > P9_40 </td>
    <td align="left"   > AIN1 </td>
    <td align="center" > </td>
    <td align="right"  > PRU1_r30_06 </td>
    <td align="right"  > P8_39 </td>
    <td align="left"   > P8_40 </td>
    <td align="left"   > PRU1_r30_07 </td>
  </tr>
  <tr>
    <td align="right"  > PRU0_r31_06 </td>
    <td align="right"  > P9_41 </td>
    <td align="left"   > P9_42 </td>
    <td align="left"   > PRU0_r31_04 </td>
    <td align="center" > </td>
    <td align="right"  > PRU1_r30_04 </td>
    <td align="right"  > P8_41 </td>
    <td align="left"   > P8_42 </td>
    <td align="left"   > PRU1_r30_05 </td>
  </tr>
  <tr>
    <td align="right"  > GND </td>
    <td align="right"  > P9_43 </td>
    <td align="left"   > P9_44 </td>
    <td align="left"   > GND </td>
    <td align="center" > </td>
    <td align="right"  > PRU1_r30_02 </td>
    <td align="right"  > P8_43 </td>
    <td align="left"   > P8_44 </td>
    <td align="left"   > PRU1_r30_03 </td>
  </tr>
  <tr>
    <td align="right"  > GND </td>
    <td align="right"  > P9_45 </td>
    <td align="left"   > P9_46 </td>
    <td align="left"   > GND </td>
    <td align="center" > </td>
    <td align="right"  > PRU1_r30_00 </td>
    <td align="right"  > P8_45 </td>
    <td align="left"   > P8_46 </td>
    <td align="left"   > PRU1_r30_01 </td>
  </tr>
</table>



