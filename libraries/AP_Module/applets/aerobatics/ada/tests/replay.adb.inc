   function Run (Ctx : AP_Module.Context) return AP_Module.Status is
      P : Configuration :=
        (6000.0, 0.1, 80.0, 5.0, 25.0, 0.25, 1.0, 2.0, 2.8, 60.0, 1.0,
         0.0, 50.0, 1500.0, 0.5, 1.0, 25.0, 0.25, 15.0, 9.0, 15.0, 18.0);
      Variant : constant := TEST_VARIANT;
      N : Node_Id;
      X, Bad : Input;
      O : Output;
      Kind : Step_Kind;
      Return_Code : Status;
      W : Snapshot := (others => 0);
      Cursor, Saved_Cursor : Natural;
      Hash : Unsigned_32;
      Initial_Eval : Evaluation;
      function From_Bits is new Ada.Unchecked_Conversion (Unsigned_32, F);
      function Observation_At (I : Natural) return Input is
         N : constant F := F (I);
         R : Input;
      begin
         R.Now := F (10000 + I * 25) * 0.001;
         R.Pos := (N * 0.45, Sin (N * 0.01) * 2.0, -30.0 + Cos (N * 0.02));
         R.Gyro := (0.1, 0.2, 0.3); R.Velocity := (18.0, 0.02, -0.01);
         R.Airspeed := 18.0; R.Eas2tas := 1.0; R.Throttle := 40.0;
         Check (B.Quaternion.From_Euler (Ctx, R.Attitude, Sin (N * 0.03) * 0.2, Cos (N * 0.01) * 0.1, 0.3));
         if Variant /= 0 then
            R.Velocity.X := (if I mod 37 = 0 then -5.0 else 10.0 + F (I mod 21));
            R.Airspeed := 4.0 + F (I mod 37); R.Eas2tas := (if I mod 43 = 0 then 5.0 else 1.0);
            R.Gyro.X := Sin (N * 0.02);
         end if;
         return R;
      end Observation_At;
      procedure Assert_True (Condition : Boolean) is
      begin
         pragma Assert (Condition);
      end Assert_True;
      procedure Parse_Test (Text : String; Valid : Boolean) is
         Result : Node_Id;
      begin
         Used := 0; Schedule_Length := Text'Length; Schedule_Text (1 .. Text'Length) := Text;
         Result := Parse_Schedule (400.0, (18.0, 60.0, False));
         if Valid then Assert_True (Fault = OK and then Result /= 0);
         else Assert_True (Fault /= OK); end if;
         if Fault = S.INVALID and then not Valid then Fault := OK; end if;
      end Parse_Test;
   begin
      Aerobatics.Ctx := Ctx; Fault := OK; Used := 0;
      Check (Sleep_Ms (Ctx, 2000));
      Assert_True (Sgn (0.0) = -1.0 and then Sgn (0.000001) = 0.0 and then Sgn (0.000002) = 1.0);
      Assert_True (Constrain (2.0, 3.0, 1.0) = 1.0);
      Assert_True (Is_Nan (Min (From_Bits (16#7FC0_0000#), 1.0)));
      Assert_True (Min (1.0, From_Bits (16#7FC0_0000#)) = 1.0);
      Assert_True (Is_Nan (Roll_Get ((Kind => Entry_Roll, Angle => 0.0, others => <>), -0.1, 1.0, 60.0)));
      Assert_True (Rust_Integer (-1.9) = -1 and then Rust_Integer (1.9) = 1);
      Assert_True (Rust_Unsigned (4294967296.0) = Unsigned_32'Last);
      Assert_True (Roll_Get ((Kind => Entry_Exit_Roll, Angle => 90.0, others => <>), 0.25, 1.0, 60.0) = 45.0);
      Assert_True (Roll_Get ((Kind => Sequence_Roll, Sequence_Angle => (0.0, 180.0, 0.0, 0.0),
                             Sequence_Weight => (1.0, 2.0, 1.0, 0.0), Count => 3, others => <>), 0.5, 1.0, 60.0) = 90.0);
      Parse_Test ("loop 25", True);
      Parse_Test ("loop 25" & Character'Val (10) & "message: dangling", False);
      Parse_Test ("loop 25" & Character'Val (10) & "align_box 1", False);
      Parse_Test ("loop NaN", False); Parse_Test ("loop 0x1p2", False);
      Parse_Test ("thr_boost: maybe" & Character'Val (10) & "loop 25", False);
      Parse_Test ("name: Test" & Character'Val (13) & Character'Val (10) & Character'Val (9) & "loop 25", True);
      if Fault /= OK then return Fault; end if;
      Used := Max_Nodes;
      N := New_Node;
      Assert_True (N = 0 and then Fault = S.NO_MEMORY);
      Fault := OK;
      Parse_Test ("# UTF-8 comment: " & Character'Val (16#C3#) & Character'Val (16#A9#)
                  & Character'Val (10) & "loop" & Character'Val (16#C2#) & Character'Val (16#A0#) & "25", True);
      if Variant = 2 then
         P.Path_Scale := -1.2; P.Lookahead := 0.0; P.Ke_Rudd := 0.0;
         P.Ang_Accel := 0.0; P.Yaw_Accel := 500.0; P.Err_Corr_P := 1.5;
      end if;
      Used := 0;
      if not Read_File ("trick72.txt") then return S.UNAVAILABLE; end if;
      X := Observation_At (0);
      Initial_Eval := (Target_Speed (X, P), P.Entry_Rate, True);
      N := Parse_Schedule (400.0, Initial_Eval);
      if Fault /= OK then return Fault; end if;
      Start_Controller (N, X, P, 0.0, (if Variant = 2 then 200 else 0));
      for I in 1 .. 512 loop
         X := Observation_At (I);
         if Variant /= 0 then State.Path_T := F (I) / 520.0; end if;
         Controller_Step (N, X, P, Kind, O);
         Assert_True (Kind = Command_Step);
         if Fault /= OK then return Fault; end if;
         Log_Write ("ADCT", "Id,Thr,R,P,Y,Rudd,Perr,Pt,Dt,Ert,Qct", "fffffffffff",
                    (F (I), O.Throttle, O.Rates.X, O.Rates.Y, O.Rates.Z, O.Rudder,
                     O.Position_Error, O.Path_T, O.Dt, O.Path_Error_T, O.Q_Change_T));
         Log_Write ("ADCS", "Id,X,Y,Z,Q1,Q2,Q3,Q4,Cx,Cy,Cz,Px,Py,Pz,Ex", "fffffffffffffff",
                    (F (I), O.Track_Pos.X, O.Track_Pos.Y, O.Track_Pos.Z, O.Track_Attitude.Q1,
                     O.Track_Attitude.Q2, O.Track_Attitude.Q3, O.Track_Attitude.Q4,
                     O.Correction_Rates.X, O.Correction_Rates.Y, O.Correction_Rates.Z,
                     O.Path_Rates.X, O.Path_Rates.Y, O.Path_Rates.Z, O.Roll_Error_Rate));
         Log_Write ("ADSP", "Id,Targ,Curr,P,I,Total,Add", "fffffff", (F (I), O.Speed_Log (1), O.Speed_Log (2),
                    O.Speed_Log (3), O.Speed_Log (4), O.Speed_Log (5), O.Speed_Log (6)));
         W := (others => 0); Save_Controller (W (24 .. 59)); Cursor := 64;
         Path_Cache (N, W, Cursor, False); Saved_Cursor := Cursor; Hash := 2166136261;
         for Index in 0 .. Cursor - 1 loop
            for Byte in 0 .. 3 loop
               Hash := (Hash xor (Shift_Right (W (Index), 8 * Byte) and 255)) * 16777619;
            end loop;
         end loop;
         Log_Write ("ADHS", "Id,Lo,Hi,N", "ffff", (F (I), F (Hash and 65535), F (Shift_Right (Hash, 16)), F (Cursor)));
         Restore_Controller (W (24 .. 59));
         if I mod 17 = 0 then
            Used := 0; Initial_Eval.Announce := False; N := Parse_Schedule (400.0, Initial_Eval);
         end if;
         Cursor := 64; Path_Cache (N, W, Cursor, True); Assert_True (Cursor = Saved_Cursor);
         Check (Sleep_Ms (Ctx, 5));
      end loop;
      Controller_Step (N, X, P, Kind, O); Assert_True (Kind = Waiting);
      Bad := X; Bad.Attitude := (0.0, 0.0, 0.0, 0.0);
      Controller_Step (N, Bad, P, Kind, O); Assert_True (Fault = S.INVALID); Fault := OK;
      X := Observation_At (513); X.Pos.Z := 100.0;
      Controller_Step (N, X, P, Kind, O); Assert_True (Kind = Altitude_Abort);
      State.Path_T := 1.0; X := Observation_At (514);
      Controller_Step (N, X, P, Kind, O); Assert_True (Kind = Finished);
      Return_Code := Fault;
      if Return_Code = OK then Message (6, "AEROP Ada controller done"); end if;
      Aerobatics.Ctx := System.Null_Address; return Return_Code;
   end Run;
end Aerobatics;
