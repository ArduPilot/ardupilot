-- SPDX-License-Identifier: GPL-3.0-or-later
-- AI-assisted translation of the adjacent Rust implementation; see README.md.
with AP_Module.Bindings;
with AP_Module.Types;
with AP_Module.Statuses;
with Interfaces; use Interfaces;
with Interfaces.C;
with Ada.Unchecked_Conversion;
with System;
package body Aerobatics is
   use AP_Module;
   use type Interfaces.C.C_float;
   use type Interfaces.C.int;
   use type Interfaces.C.C_bool;
   package B renames AP_Module.Bindings;
   package T renames AP_Module.Types;
   package S renames AP_Module.Statuses;
   subtype F is Interfaces.C.C_float;
   subtype V is T.Vector3f;
   subtype Q is T.Quaternion;
   type Floats is array (Natural range <>) of F;
   subtype Args_Type is Floats (1 .. 4);
   Pi : constant F := 3.1415927410125732421875;
   Identity : constant Q := (1.0, 0.0, 0.0, 0.0);
   Zero : constant V := (0.0, 0.0, 0.0);
   Ctx : Context := System.Null_Address;
   Fault : Status := OK;
   function Rad (X : F) return F is (X * (Pi / 180.0));
   function Deg (X : F) return F is (X * (180.0 / Pi));
   function Min (A, D : F) return F is (if D < A then D else A);
   function Max (A, D : F) return F is (if D > A then D else A);
   function Constrain (X, Low, High : F) return F is
      R : F := X;
   begin
      if R < Low then R := Low; end if;
      if R > High then R := High; end if;
      return R;
   end Constrain;
   function Sgn (X : F) return F is
     (if X > 0.000001 then 1.0 elsif X < 0.000001 then -1.0 else 0.0);
   function Bits is new Ada.Unchecked_Conversion (F, Unsigned_32);
   function From_Bits is new Ada.Unchecked_Conversion (Unsigned_32, F);
   type Words is array (Natural range <>) of Unsigned_32;
   subtype Snapshot is Words (0 .. 511);
   subtype Controller_Words is Words (0 .. 35);
   function Finite (X : F) return Boolean is ((Bits (X) and 16#7F80_0000#) /= 16#7F80_0000#);
   function Finite (X : V) return Boolean is (Finite (X.X) and Finite (X.Y) and Finite (X.Z));
   function Is_Nan (X : F) return Boolean is
     ((Bits (X) and 16#7FFF_FFFF#) > 16#7F80_0000#);
   function Is_Nan (X : V) return Boolean is (Is_Nan (X.X) or Is_Nan (X.Y) or Is_Nan (X.Z));
   procedure Check (Code : Status) is
   begin
      if Fault = OK then Fault := Code; end if;
   end Check;
   procedure Invalid is
   begin
      Check (S.INVALID);
   end Invalid;
   function Raw_Time_Us (Ctx : Context) return Unsigned_64
     with Import, Convention => C, External_Name => "ap_module_aerobatics_time_us";
   function Time_Us return Unsigned_64 is (Raw_Time_Us (Ctx));
   function Elapsed (Now, Before : Unsigned_64) return Unsigned_64 is
     (if Now >= Before then Now - Before else 0);
   procedure Message (Level : Integer; Text : String) is
      Code : Status;
   begin
      Code := B.Gcs.Send_Text (Ctx, T.MavSeverity (Level), Text);
      if Code = Stopped then Check (Code); end if;
   end Message;
   type Number_Text is record
      Data : String (1 .. 10);
      First : Positive;
   end record;
   function Decimal (Value : Natural) return Number_Text is
      -- A fixed result avoids Ada runtime-dependent integer Image support.
      Buffer : String (1 .. 10) := (others => '0');
      N : Natural := Value;
      First : Positive := Buffer'Last;
   begin
      loop
         Buffer (First) := Character'Val (Character'Pos ('0') + N mod 10);
         N := N / 10;
         exit when N = 0;
         First := First - 1;
      end loop;
      return (Buffer, First);
   end Decimal;
   function Sin (X : F) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Libm.Sinf (Ctx, X, R)); end if;
      return R;
   end Sin;
   function Cos (X : F) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Libm.Cosf (Ctx, X, R)); end if;
      return R;
   end Cos;
   function Sqrt (X : F) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Libm.Sqrtf (Ctx, X, R)); end if;
      return R;
   end Sqrt;
   function Floor (X : F) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Libm.Floorf (Ctx, X, R)); end if;
      return R;
   end Floor;
   function Atan2 (X : F; Y : F) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Libm.Atan2f (Ctx, X, Y, R)); end if;
      return R;
   end Atan2;
   function Fmod (X : F; Y : F) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Libm.Fmodf (Ctx, X, Y, R)); end if;
      return R;
   end Fmod;
   function Add (X : V; Y : V) return V is
      R : V := Zero;
   begin
      if Fault = OK then Check (B.Vector3f.Add (Ctx, X, Y, R)); end if;
      return R;
   end Add;
   function Sub (X : V; Y : V) return V is
      R : V := Zero;
   begin
      if Fault = OK then Check (B.Vector3f.Sub (Ctx, X, Y, R)); end if;
      return R;
   end Sub;
   function Scale (X : V; Y : F) return V is
      R : V := Zero;
   begin
      if Fault = OK then Check (B.Vector3f.Scale (Ctx, X, Y, R)); end if;
      return R;
   end Scale;
   function Dot (X : V; Y : V) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Vector3f.Dot (Ctx, X, Y, R)); end if;
      return R;
   end Dot;
   function Cross (X : V; Y : V) return V is
      R : V := Zero;
   begin
      if Fault = OK then Check (B.Vector3f.Cross (Ctx, X, Y, R)); end if;
      return R;
   end Cross;
   function Length (X : V) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Vector3f.Length (Ctx, X, R)); end if;
      return R;
   end Length;
   function Length (X : Q) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Quaternion.Length (Ctx, X, R)); end if;
      return R;
   end Length;
   function Mul (X : Q; Y : Q) return Q is
      R : Q := Identity;
   begin
      if Fault = OK then Check (B.Quaternion.Mul (Ctx, X, Y, R)); end if;
      return R;
   end Mul;
   function Inverse (X : Q) return Q is
      R : Q := Identity;
   begin
      if Fault = OK then Check (B.Quaternion.Inverse (Ctx, X, R)); end if;
      return R;
   end Inverse;
   function Euler_Roll (X : Q) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Quaternion.Get_Euler_Roll (Ctx, X, R)); end if;
      return R;
   end Euler_Roll;
   function Euler_Pitch (X : Q) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Quaternion.Get_Euler_Pitch (Ctx, X, R)); end if;
      return R;
   end Euler_Pitch;
   function Euler_Yaw (X : Q) return F is
      R : F := 0.0;
   begin
      if Fault = OK then Check (B.Quaternion.Get_Euler_Yaw (Ctx, X, R)); end if;
      return R;
   end Euler_Yaw;
   function Wrap_360 (X : F) return F is
      R : F := Fmod (X, 360.0);
   begin
      if R < 0.0 then R := R + 360.0; end if;
      return R;
   end Wrap_360;
   function Wrap_180 (X : F) return F is
      R : constant F := Wrap_360 (X);
   begin
      return (if R > 180.0 then R - 360.0 else R);
   end Wrap_180;
   function Wrap_Pi (X : F) return F is (Rad (Wrap_180 (Deg (X))));
   function Wrap_2pi (X : F) return F is (Rad (Wrap_360 (Deg (X))));
   procedure Normalize (X : in out Q) is
      L : constant F := Length (X);
   begin
      if not Finite (L) or else L < 1.1920928955078125E-7 then Invalid; end if;
      if Fault = OK then Check (B.Quaternion.Normalize (Ctx, X)); end if;
   end Normalize;
   function Unit (X : V) return V is
      R : V := X;
      L : constant F := Length (X);
   begin
      if not Finite (L) or else L <= 0.0 then Invalid; end if;
      if Fault = OK then Check (B.Vector3f.Normalize (Ctx, R)); end if;
      return R;
   end Unit;
   function Orient (Roll, Pitch, Yaw : F) return Q is
      R : Q := Identity;
   begin
      if Fault = OK then Check (B.Quaternion.From_Euler (Ctx, R, Rad (Roll), Rad (Pitch), Rad (Yaw))); end if;
      return R;
   end Orient;
   function Axis_Angle (Axis : V; Angle : F) return Q is
      R : Q := Identity;
   begin
      if Fault = OK then Check (B.Quaternion.From_Axis_Angle (Ctx, R, Axis, Angle)); end if;
      return R;
   end Axis_Angle;
   function Angular_Velocity (X : V; Dt : F) return Q is
      R : Q := Identity;
      Half : constant F := 0.5 * Dt;
      A : constant V := (Half * X.X, Half * X.Y, Half * X.Z);
   begin
      if not Finite (1.0 + A.X * A.X + A.Y * A.Y + A.Z * A.Z) then Invalid; end if;
      if Fault = OK then Check (B.Quaternion.From_Angular_Velocity (Ctx, R, X, Dt)); end if;
      return R;
   end Angular_Velocity;
   function Earth_To_Body (Orientation : Q; X : V) return V is
      R : V := X;
   begin
      if Fault = OK then Check (B.Quaternion.Earth_To_Body (Ctx, Orientation, R)); end if;
      return R;
   end Earth_To_Body;
   function Body_To_Earth (Orientation : Q; X : V) return V is (Earth_To_Body (Inverse (Orientation), X));
   function To_Axis_Angle (Orientation : Q) return V is
      R : V := Zero;
   begin
      if Fault = OK then Check (B.Quaternion.To_Axis_Angle (Ctx, Orientation, R)); end if;
      return R;
   end To_Axis_Angle;
   function Tangents_To_Rate (A, D : V; Dt : F) return V is
      X, Y, Axis : V;
      Rot : Q;
      L, Al, Bl : F;
   begin
      if not Finite (Dt) or else Dt <= 0.0 or else not Finite (1.0 / Dt) then Invalid; return Zero; end if;
      Al := Length (A); Bl := Length (D);
      if not Finite (Al) or else not Finite (Bl) or else Al <= 0.0 or else Bl <= 0.0 then
         Invalid; return Zero;
      end if;
      X := Unit (A); Y := Unit (D); Axis := Cross (X, Y);
      Rot := (1.0 + Dot (X, Y), Axis.X, Axis.Y, Axis.Z);
      L := Length (Rot);
      if not Finite (L) or else L < 1.1920928955078125E-7 then Invalid; return Zero; end if;
      Normalize (Rot);
      return Scale (To_Axis_Angle (Rot), 1.0 / Dt);
   end Tangents_To_Rate;
   function Ortho (X, Tangent : V) return V is (Cross (Tangent, Cross (Tangent, X)));

   type Optional_F is record
      Present : Boolean := False;
      Value : F := 0.0;
   end record;
   type Optional_Q is record
      Present : Boolean := False;
      Value : Q := Identity;
   end record;
   type Optional_Bool is record
      Present : Boolean := False;
      Value : Boolean := False;
   end record;
   type Attributes is record
      Roll_Ref, Pos_Corr, Pos_Gain_Mul : Optional_F;
      Set_Orient : Optional_Q;
      Thr_Boost, Shift_Xy : Optional_Bool;
   end record;
   No_Attributes : constant Attributes :=
     (Roll_Ref | Pos_Corr | Pos_Gain_Mul => (False, 0.0), Set_Orient => (False, (1.0, 0.0, 0.0, 0.0)),
      Thr_Boost | Shift_Xy => (False, False));
   type Roll_Kind is (Angle_Roll, Entry_Exit_Roll, Entry_Roll, Exit_Roll, Sequence_Roll);
   type Roll_Type is record
      Kind : Roll_Kind := Angle_Roll;
      Angle : F := 0.0;
      -- AirShow's compiled sequence profiles use three equal weights.
      Sequence_Angle : Args_Type := (others => 0.0);
      Sequence_Weight : Args_Type := (others => 0.0);
      Count : Natural range 0 .. 4 := 0;
   end record;
   function Roll_Get (R : Roll_Type; Time, Duration, Entry_Rate : F) return F is
      Entry_T, Total, Start_T, End_T, Start_Angle : F;
   begin
      case R.Kind is
         when Angle_Roll => return (if R.Angle = 0.0 then 0.0 else R.Angle * Time);
         when Entry_Exit_Roll =>
            Entry_T := Min ((abs R.Angle / Entry_Rate) / Duration, 0.5);
            if Time <= 0.0 then return 0.0;
            elsif Time < Entry_T then return R.Angle * Time / Entry_T;
            elsif Time < 1.0 - Entry_T then return R.Angle;
            elsif R.Angle = 0.0 or else Time >= 1.0 then return 0.0;
            else return (1.0 - (Time - (1.0 - Entry_T)) / Entry_T) * R.Angle; end if;
         when Entry_Roll =>
            Entry_T := Min ((abs R.Angle / Entry_Rate) / Duration, 0.5);
            return (if Time < Entry_T then R.Angle * Time / Entry_T else R.Angle);
         when Exit_Roll =>
            Entry_T := (abs R.Angle / Entry_Rate) / Duration;
            return (if Time < 1.0 - Entry_T or else R.Angle = 0.0 then 0.0
                    else ((Time - (1.0 - Entry_T)) / Entry_T) * R.Angle);
         when Sequence_Roll =>
            if R.Count = 0 then Invalid; return 0.0; end if;
            Total := 0.0;
            for I in 1 .. R.Count loop Total := Total + R.Sequence_Weight (I); end loop;
            Start_T := 0.0; Start_Angle := 0.0;
            for I in 1 .. R.Count loop
               End_T := Start_T + R.Sequence_Weight (I) / Total;
               if Time <= End_T then
                  return Start_Angle + ((Time - Start_T) / (R.Sequence_Weight (I) / Total)) * R.Sequence_Angle (I);
               end if;
               Start_T := End_T; Start_Angle := Start_Angle + R.Sequence_Angle (I);
            end loop;
            return Start_Angle;
      end case;
   end Roll_Get;
   type Geometry_Kind is (Straight_G, Reverse_G, Vertical_G, Horizontal_G, Cylinder_G);
   type Geometry is record
      Kind : Geometry_Kind := Straight_G;
      Radius, Angle, Height, Distance, Spirals, Gamma : F := 0.0;
   end record;
   function Position (G : Geometry; Time : F) return V is
      A : F;
      R : V;
      Rot : Q;
   begin
      case G.Kind is
         when Straight_G => return (G.Distance * Time, 0.0, 0.0);
         when Reverse_G =>
            return ((if Time < 0.5 then G.Distance * Time * 2.0
                     else G.Distance - G.Distance * (Time - 0.5) * 2.0), 0.0, 0.0);
         when Vertical_G =>
            A := Wrap_2pi (Time * Rad (G.Angle));
            return (abs G.Radius * Sin (A), 0.0, -G.Radius * (1.0 - Cos (A)));
         when Horizontal_G =>
            A := Time * Rad (G.Angle);
            return (abs G.Radius * Sin (A), G.Radius * (1.0 - Cos (A)), -G.Height * Time);
         when Cylinder_G =>
            A := Time * G.Spirals * Pi * 2.0;
            R := (G.Distance * Time, abs G.Radius * Sin (A + Pi), -abs G.Radius * (1.0 - Cos (A)));
            Rot := Axis_Angle ((0.0, 0.0, 1.0), 0.5 * Pi - G.Gamma);
            R := Earth_To_Body (Rot, R);
            if G.Radius < 0.0 then R.Y := -R.Y; end if;
            return R;
      end case;
   end Position;
   function Geometry_Length (G : Geometry) return F is
      Circumference : constant F := 2.0 * Pi * abs G.Radius;
      H, D : F;
   begin
      case G.Kind is
         when Straight_G => return G.Distance;
         when Reverse_G => return G.Distance * 2.0;
         when Vertical_G => return abs G.Radius * 2.0 * Pi * abs G.Angle / 360.0;
         when Horizontal_G =>
            H := G.Height * 360.0 / abs G.Angle;
            return Sqrt (H * H + Circumference * Circumference) * abs G.Angle / 360.0;
         when Cylinder_G =>
            D := G.Distance / G.Spirals;
            return Sqrt (D * D + Circumference * Circumference) * G.Spirals;
      end case;
   end Geometry_Length;
   function Final_Orientation (G : Geometry) return Q is
      R : Q := Identity;
   begin
      if G.Kind = Vertical_G then
         R := Axis_Angle ((0.0, 1.0, 0.0), Sgn (G.Radius) * Rad (Wrap_180 (G.Angle)));
         Normalize (R);
      elsif G.Kind = Horizontal_G then
         R := Axis_Angle ((0.0, 0.0, 1.0), Sgn (G.Radius) * Rad (G.Angle));
      end if;
      return R;
   end Final_Orientation;
   function Correction (G : Geometry; Time : F) return F is
      Gamma : F;
   begin
      if G.Kind = Horizontal_G and then G.Height /= 0.0 then
         Gamma := Atan2 (G.Height * (360.0 / G.Angle) / (2.0 * Pi * G.Radius), 1.0);
         return -Time * G.Angle * Sin (Gamma);
      elsif G.Kind = Cylinder_G then
         return Sgn (G.Radius) * Time * 360.0 * Sin (G.Gamma) * G.Spirals;
      end if;
      return 0.0;
   end Correction;

   -- The runtime-free Ada subset uses a bounded arena instead of heap objects.
   Max_Nodes : constant := 128;
   subtype Node_Id is Natural range 0 .. Max_Nodes;
   type Path_Kind is (Leaf_Path, Align_Path, Composite_Path);
   type Node is record
      Kind : Path_Kind := Leaf_Path;
      G : Geometry;
      Roll : Roll_Type;
      Attr : Attributes;
      Message : String (1 .. 50) := (others => ' ');
      Message_Length : Natural range 0 .. 50 := 0;
      Length : F := 0.0;
      Final_Orientation : Q := Identity;
      Have_Extents : Boolean := False;
      Low, High : F := 0.0;
      Rebuild, Speed_Dependent : Boolean := False;
      Bank_Radius : F := 0.0;
      Alignment : Optional_F;
      First, Next, Last, Highest, Last_Index : Node_Id := 0;
      Start_Time : F := 0.0;
      Start_Pos : V := Zero;
      Start_Orientation : Q := Identity;
      Start_Angle, Start_Correction : F := 0.0;
   end record;
   pragma Suppress_Initialization (Node);
   Nodes : array (Node_Id range 0 .. Max_Nodes) of Node;
   Used : Node_Id := 0;
   type Evaluation is record
      Speed, Entry_Rate : F;
      Announce : Boolean;
   end record;
   function New_Node return Node_Id is
   begin
      if Fault /= OK then return 0; end if;
      if Used = Max_Nodes then Check (S.NO_MEMORY); return 0; end if;
      Used := Used + 1;
      Nodes (Used) := (others => <>);
      return Used;
   end New_Node;
   function Leaf (G : Geometry; Roll : Roll_Type) return Node_Id is
      N : constant Node_Id := New_Node;
   begin
      if N = 0 then return 0; end if;
      Nodes (N).G := G; Nodes (N).Roll := Roll;
      Nodes (N).Length := Geometry_Length (G);
      Nodes (N).Final_Orientation := Final_Orientation (G);
      return N;
   end Leaf;
   procedure Append (First, Last : in out Node_Id; Child : Node_Id) is
   begin
      if Child = 0 or else Fault /= OK then return; end if;
      if First = 0 then First := Child; else Nodes (Last).Next := Child; end if;
      Last := Child;
   end Append;
   function Path_Position (N : Node_Id; Time : F; E : Evaluation) return V;
   function Path_Roll (N : Node_Id; Time, Duration : F; E : Evaluation) return F;
   function Path_Correction (N : Node_Id; Time : F; E : Evaluation) return F;
   procedure Extents (N : Node_Id; E : Evaluation);
   function Barrel_Bank (Radius, Speed : F) return F is
     (Deg (Atan2 ((Speed * Speed) / (Radius * 9.80665), 1.0)));
   procedure Activate (N : Node_Id; Speed : F) is
      Bank : F;
      First, Middle, Last : Node_Id;
   begin
      if Nodes (N).Kind /= Composite_Path then return; end if;
      Nodes (N).Highest := 0; Nodes (N).Last_Index := Nodes (N).Last;
      if Nodes (N).Speed_Dependent then
         Bank := Barrel_Bank (Nodes (N).Bank_Radius, Speed);
         First := Nodes (N).First; Middle := Nodes (First).Next; Last := Nodes (Middle).Next;
         if Last = 0 or else Nodes (Last).Next /= 0
           or else Nodes (First).Kind /= Leaf_Path or else Nodes (Last).Kind /= Leaf_Path
         then Invalid; return; end if;
         Nodes (First).Roll := (Kind => Entry_Exit_Roll, Angle => -Bank, others => <>);
         Nodes (Last).Roll := (Kind => Entry_Exit_Roll, Angle => Bank, others => <>);
      end if;
   end Activate;
   procedure Path_Cache (N : Node_Id; W : in out Snapshot; Cursor : in out Natural; Restoring : Boolean) is
      Child, Last_Child : Node_Id;
      Count, Last_Index : Natural := 0;
      Found : Boolean := False;
      Flags : Unsigned_32;
      Bank : F := 0.0;
   begin
      if Fault /= OK or else N = 0 or else Nodes (N).Kind /= Composite_Path then return; end if;
      if Cursor + 4 > W'Length then Invalid; return; end if;
      Child := Nodes (N).First;
      while Child /= 0 loop
         if Child = Nodes (N).Last_Index then Last_Index := Count; Found := True; end if;
         Count := Count + 1; Child := Nodes (Child).Next;
      end loop;
      Flags := Unsigned_32 (Count) + (if Nodes (N).Speed_Dependent then 65536 else 0);
      if Restoring then
         Bank := From_Bits (W (Cursor + 3));
         if W (Cursor) /= Flags or else W (Cursor + 1) > Unsigned_32 (Count)
           or else W (Cursor + 2) >= Unsigned_32 (Count) or else not Finite (Bank)
         then Invalid; return; end if;
         Nodes (N).Highest := Node_Id (W (Cursor + 1)); Child := Nodes (N).First;
         for I in 1 .. Natural (W (Cursor + 2)) loop Child := Nodes (Child).Next; end loop;
         Nodes (N).Last_Index := Child;
         if Nodes (N).Speed_Dependent then
            Nodes (Nodes (N).First).Roll.Angle := -Bank;
            Last_Child := Nodes (N).Last; Nodes (Last_Child).Roll.Angle := Bank;
         end if;
      else
         if not Found then Invalid; return; end if;
         if Nodes (N).Speed_Dependent then Bank := -Nodes (Nodes (N).First).Roll.Angle; end if;
         W (Cursor .. Cursor + 3) := (Flags, Unsigned_32 (Nodes (N).Highest), Unsigned_32 (Last_Index), Bits (Bank));
      end if;
      Cursor := Cursor + 4; Child := Nodes (N).First;
      while Child /= 0 loop
         Path_Cache (Child, W, Cursor, Restoring); Child := Nodes (Child).Next;
      end loop;
   end Path_Cache;
   procedure Segment (N : Node_Id; Time : F; E : Evaluation; Child : out Node_Id; Sub_T : out F) is
      Index : Node_Id := 1;
   begin
      Child := Nodes (N).First; Sub_T := 0.0;
      if Fault /= OK then return; end if;
      if not Finite (E.Speed) or else E.Speed <= 0.0
        or else not Finite (E.Entry_Rate) or else E.Entry_Rate <= 0.0 then Invalid; return; end if;
      while Nodes (Child).Next /= 0
        and then Time >= Nodes (Child).Start_Time + Nodes (Child).Length / Nodes (N).Length
      loop
         Child := Nodes (Child).Next; Index := Index + 1;
      end loop;
      if Nodes (N).Last_Index /= Child then
         if Nodes (Child).Rebuild then Activate (Child, E.Speed); end if;
         Nodes (N).Last_Index := Child;
      end if;
      Sub_T := (Time - Nodes (Child).Start_Time) / (Nodes (Child).Length / Nodes (N).Length);
      if Index > Nodes (N).Highest and then Time > 0.0 and then Time < 1.0 then
         if E.Announce and then Nodes (Child).Message_Length > 0 then
            Message (1, Nodes (Child).Message (1 .. Nodes (Child).Message_Length));
         end if;
         Nodes (N).Highest := Index;
      end if;
   end Segment;
   function Path_Position (N : Node_Id; Time : F; E : Evaluation) return V is
      Child : Node_Id;
      Sub_T : F;
      Point : V;
   begin
      if Fault /= OK then return Zero; end if;
      case Nodes (N).Kind is
         when Leaf_Path => return Position (Nodes (N).G, Time);
         when Align_Path => Invalid; return Zero;
         when Composite_Path =>
            Segment (N, Time, E, Child, Sub_T);
            Point := Path_Position (Child, Sub_T, E);
            return Add (Earth_To_Body (Nodes (Child).Start_Orientation, Point), Nodes (Child).Start_Pos);
      end case;
   end Path_Position;
   function Path_Roll (N : Node_Id; Time, Duration : F; E : Evaluation) return F is
      Child : Node_Id;
      Sub_T : F;
   begin
      if Fault /= OK then return 0.0; end if;
      case Nodes (N).Kind is
         when Leaf_Path => return Wrap_180 (Roll_Get (Nodes (N).Roll, Time, Duration, E.Entry_Rate));
         when Align_Path => Invalid; return 0.0;
         when Composite_Path =>
            Segment (N, Time, E, Child, Sub_T);
            return Path_Roll (Child, Sub_T, Nodes (Child).Length / E.Speed, E) + Nodes (Child).Start_Angle;
      end case;
   end Path_Roll;
   function Path_Correction (N : Node_Id; Time : F; E : Evaluation) return F is
      Child : Node_Id;
      Sub_T : F;
   begin
      if Fault /= OK then return 0.0; end if;
      case Nodes (N).Kind is
         when Leaf_Path => return Correction (Nodes (N).G, Time);
         when Align_Path => Invalid; return 0.0;
         when Composite_Path =>
            Segment (N, Time, E, Child, Sub_T);
            return Path_Correction (Child, Sub_T, E) + Nodes (Child).Start_Correction;
      end case;
   end Path_Correction;
   procedure Extents (N : Node_Id; E : Evaluation) is
      X, Time : F;
   begin
      if Nodes (N).Have_Extents or else Fault /= OK then return; end if;
      X := Path_Position (N, 0.0, E).X;
      Nodes (N).Low := X; Nodes (N).High := X;
      Time := 0.0;
      while Time <= 1.0 and then Fault = OK loop
         X := Path_Position (N, Time, E).X;
         Nodes (N).Low := Min (Nodes (N).Low, X); Nodes (N).High := Max (Nodes (N).High, X);
         Time := Time + 0.02;
      end loop;
      Nodes (N).Have_Extents := Fault = OK;
   end Extents;
   function Merge (Outer, Inner : Attributes) return Attributes is
      R : Attributes := Inner;
   begin
      if Outer.Roll_Ref.Present then R.Roll_Ref := Outer.Roll_Ref; end if;
      if Outer.Set_Orient.Present then R.Set_Orient := Outer.Set_Orient; end if;
      if Outer.Pos_Corr.Present then R.Pos_Corr := Outer.Pos_Corr; end if;
      if Outer.Pos_Gain_Mul.Present then R.Pos_Gain_Mul := Outer.Pos_Gain_Mul; end if;
      if Outer.Thr_Boost.Present and then Outer.Thr_Boost.Value then R.Thr_Boost := Outer.Thr_Boost; end if;
      if Outer.Shift_Xy.Present and then Outer.Shift_Xy.Value then R.Shift_Xy := Outer.Shift_Xy; end if;
      return R;
   end Merge;
   function Path_Attributes (N : Node_Id; Time : F; E : Evaluation) return Attributes is
      Child : Node_Id;
      Sub_T : F;
      Inner : Attributes;
   begin
      if Fault /= OK then return No_Attributes; end if;
      if Nodes (N).Kind /= Composite_Path then return Nodes (N).Attr; end if;
      Segment (N, Time, E, Child, Sub_T);
      Inner := Path_Attributes (Child, Sub_T, E);
      return Merge (Nodes (Child).Attr, Inner);
   end Path_Attributes;
   function Next_Segment_Start (N : Node_Id; Time : F; E : Evaluation) return F is
      Child : Node_Id;
      Sub_T, Fraction, End_Time : F;
   begin
      if Nodes (N).Kind /= Composite_Path or else Fault /= OK then return 1.0; end if;
      Segment (N, Time, E, Child, Sub_T);
      Fraction := Nodes (Child).Length / Nodes (N).Length;
      End_Time := Nodes (Child).Start_Time + Fraction;
      if Nodes (Child).Kind = Composite_Path then
         return Nodes (Child).Start_Time + Next_Segment_Start (Child, Sub_T, E) * (End_Time - Nodes (Child).Start_Time);
      end if;
      return End_Time;
   end Next_Segment_Start;
   function Compose (First, Last : Node_Id; Width : F; E : Evaluation) return Node_Id is
      N : Node_Id;
      Child : Node_Id := First;
      Rot : Q := Identity;
      Pos : V := Zero;
      Angle, Corr, Total, Time, Distance, Start_X : F := 0.0;
      Point : V;
   begin
      if Fault /= OK then return 0; end if;
      if First = 0 or else not Finite (E.Speed) or else E.Speed <= 0.0
        or else not Finite (E.Entry_Rate) or else E.Entry_Rate <= 0.0 or else not Finite (Width)
      then Invalid; return 0; end if;
      while Child /= 0 and then Fault = OK loop
         if Nodes (Child).Kind = Align_Path then
            if Nodes (Child).Next = 0 then Invalid; return 0; end if;
            Extents (Nodes (Child).Next, E);
            declare
               Following : Node renames Nodes (Nodes (Child).Next);
            begin
               Start_X := (if abs Deg (Euler_Yaw (Rot)) > 90.0 then Pos.X else -Pos.X);
               Distance := (if Nodes (Child).Alignment.Present
                            then (Width / 2.0) * Nodes (Child).Alignment.Value + Start_X - Following.High
                            else Start_X - (Following.Low + Following.High) * 0.5);
            end;
            Distance := Max (Distance, 0.01);
            Nodes (Child).Kind := Leaf_Path;
            Nodes (Child).G := (Kind => Straight_G, Distance => Distance, others => <>);
            Nodes (Child).Roll := (others => <>);
            Nodes (Child).Length := Distance; Nodes (Child).Rebuild := False;
         end if;
         if not Finite (Nodes (Child).Length) or else Nodes (Child).Length < 0.0 then Invalid; return 0; end if;
         Nodes (Child).Start_Pos := Pos; Nodes (Child).Start_Orientation := Rot;
         Nodes (Child).Start_Angle := Angle; Nodes (Child).Start_Correction := Corr;
         Total := Total + Nodes (Child).Length;
         Point := Path_Position (Child, 1.0, E);
         Pos := Add (Pos, Earth_To_Body (Rot, Point));
         Rot := Mul (Nodes (Child).Final_Orientation, Rot); Normalize (Rot);
         Angle := Angle + Path_Roll (Child, 1.0, Nodes (Child).Length / E.Speed, E);
         Corr := Corr + Path_Correction (Child, 1.0, E);
         if Nodes (Child).Attr.Set_Orient.Present then Rot := Nodes (Child).Attr.Set_Orient.Value; end if;
         if Nodes (Child).Attr.Roll_Ref.Present then
            Rot := Mul (Rot, Axis_Angle ((1.0, 0.0, 0.0), Rad (Nodes (Child).Attr.Roll_Ref.Value)));
            Normalize (Rot);
         end if;
         Child := Nodes (Child).Next;
      end loop;
      if not Finite (Total) or else Total <= 0.0 or else Nodes (Last).Length <= 0.0 then Invalid; return 0; end if;
      N := New_Node;
      if N = 0 then return 0; end if;
      Nodes (N).Kind := Composite_Path; Nodes (N).First := First; Nodes (N).Last := Last;
      Nodes (N).Last_Index := Last; Nodes (N).Length := Total;
      Nodes (N).Final_Orientation := Mul (Axis_Angle ((1.0, 0.0, 0.0), Rad (Wrap_180 (Angle))), Rot);
      Child := First;
      while Child /= 0 loop
         Nodes (Child).Start_Time := Time;
         Time := Time + Nodes (Child).Length / Total;
         Child := Nodes (Child).Next;
      end loop;
      return N;
   end Compose;
   type Command is (Loop_C, Half_Reverse_Cuban, Scale_Figure_Eight, Immelmann,
                    Straight_Roll, Split_S, Rolling_Circle, Humpty_Bump, Half_Cuban,
                    Barrel_Roll, Crossbox_Tophat, Triangular_Loop, Align_Box, Align_Center, Unknown);
   function Builtin (Id : Integer) return Command is
   begin
      case Id is
         when 2 => return Loop_C;
         when 7 => return Straight_Roll;
         when 8 => return Rolling_Circle;
         when 9 => return Half_Cuban;
         when 10 => return Half_Reverse_Cuban;
         when 12 => return Humpty_Bump;
         when 14 => return Scale_Figure_Eight;
         when 15 => return Immelmann;
         when 16 => return Split_S;
         when 26 => return Barrel_Roll;
         when others => return Unknown;
      end case;
   end Builtin;
   function Named (Name : String) return Command is
   begin
      if Name = "loop" then return Loop_C;
      elsif Name = "half_reverse_cuban_eight" then return Half_Reverse_Cuban;
      elsif Name = "scale_figure_eight" then return Scale_Figure_Eight;
      elsif Name = "immelmann_turn" then return Immelmann;
      elsif Name = "straight_roll" then return Straight_Roll;
      elsif Name = "split_s" then return Split_S;
      elsif Name = "rolling_circle" then return Rolling_Circle;
      elsif Name = "humpty_bump" then return Humpty_Bump;
      elsif Name = "half_cuban_eight" then return Half_Cuban;
      elsif Name = "barrel_roll" then return Barrel_Roll;
      elsif Name = "crossbox_tophat" then return Crossbox_Tophat;
      elsif Name = "triangular_loop" then return Triangular_Loop;
      elsif Name = "align_box" then return Align_Box;
      elsif Name = "align_center" then return Align_Center;
      else return Unknown; end if;
   end Named;
   function Build (Cmd : Command; Args : Args_Type; Width : F; E : Evaluation) return Node_Id is
      R : constant F := Args (1);
      A : constant F := abs R;
      First, Last, N, Child : Node_Id := 0;
      Loops, Gamma, Bank, R2, H3, Side, Base, Corner : F;
      procedure Add_Piece (G : Geometry; Roll : Roll_Type) is
      begin
         Child := Leaf (G, Roll); Append (First, Last, Child);
      end Add_Piece;
      procedure Straight (Distance, Roll : F) is
      begin
         Add_Piece ((Kind => Straight_G, Distance => Distance, others => <>),
                    (Kind => Angle_Roll, Angle => Roll, others => <>));
      end Straight;
      procedure Vertical (Radius, Angle : F) is
      begin
         Add_Piece ((Kind => Vertical_G, Radius => Radius, Angle => Angle, others => <>), (others => <>));
      end Vertical;
      procedure Horizontal (Radius, Angle, Bank : F) is
      begin
         Add_Piece ((Kind => Horizontal_G, Radius => Radius, Angle => Angle, others => <>),
                    (Kind => Entry_Exit_Roll, Angle => Bank, others => <>));
      end Horizontal;
      procedure Set_Orient (Roll, Pitch, Yaw : F) is
      begin
         Nodes (Child).Attr.Set_Orient := (True, Orient (Roll, Pitch, Yaw));
      end Set_Orient;
      procedure Sequence_Straight (Distance, Angle : F) is
      begin
         Add_Piece ((Kind => Straight_G, Distance => Distance, others => <>),
                    (Kind => Sequence_Roll, Sequence_Angle => (0.0, Angle, 0.0, 0.0),
                     Sequence_Weight => (1.0, 1.0, 1.0, 0.0), Count => 3, others => <>));
      end Sequence_Straight;
   begin
      if Fault /= OK then return 0; end if;
      for Arg of Args loop
         if not Finite (Arg) then Invalid; return 0; end if;
      end loop;
      if not Finite (E.Speed) or else E.Speed <= 0.0 or else not Finite (E.Entry_Rate)
        or else E.Entry_Rate <= 0.0 then Invalid; return 0; end if;
      case Cmd is
         when Align_Box | Align_Center =>
            N := New_Node;
            if N /= 0 then
               Nodes (N).Kind := Align_Path;
               Nodes (N).Alignment := (Cmd = Align_Box, R);
            end if;
            return N;
         when Loop_C =>
            Loops := (if Args (3) <= 0.0 then 1.0 else Args (3));
            Add_Piece ((Kind => Vertical_G, Radius => R, Angle => 360.0 * Loops, others => <>),
                       (Kind => Entry_Exit_Roll, Angle => Args (2), others => <>));
         when Half_Reverse_Cuban =>
            Vertical (R, 45.0); Straight (2.0 * A / 3.0, 0.0);
            Straight (2.0 * A / 3.0, 180.0); Straight (2.0 * A / 3.0, 0.0); Vertical (-R, 225.0);
         when Half_Cuban =>
            Straight (2.0 * A * Sqrt (2.0), 0.0); Vertical (R, 225.0);
            Straight (2.0 * A / 3.0, 0.0); Straight (2.0 * A / 3.0, 180.0);
            Straight (2.0 * A / 3.0, 0.0); Vertical (-R, 45.0);
         when Scale_Figure_Eight =>
            Straight (A, 0.0); Horizontal (R, 90.0, Args (2));
            Horizontal (-R, 360.0, -Args (2)); Horizontal (R, 270.0, Args (2)); Straight (3.0 * A, 0.0);
         when Immelmann => Vertical (R, 180.0); Straight (A, 180.0);
         when Straight_Roll => Straight (R, Args (2) * 360.0);
         when Split_S => Straight (A / 2.0, 180.0); Vertical (-R, 180.0);
         when Rolling_Circle =>
            Add_Piece ((Kind => Horizontal_G, Radius => R, Angle => 360.0, others => <>),
                       (Kind => Angle_Roll, Angle => 360.0 * Args (2), others => <>));
            Nodes (Child).Attr.Thr_Boost := (True, True);
         when Humpty_Bump =>
            if Args (2) < 2.0 * R then Invalid; return 0; end if;
            Vertical (R, 90.0); Straight ((Args (2) - 2.0 * A) / 3.0, 0.0);
            Straight ((Args (2) - 2.0 * A) / 3.0, 180.0);
            Straight ((Args (2) - 2.0 * A) / 3.0, 0.0); Vertical (-R, 180.0);
            Straight (Args (2) - 2.0 * A, 0.0); Vertical (-R, 90.0); Straight (2.0 * A, 0.0);
         when Barrel_Roll =>
            if R = 0.0 or else Args (3) <= 0.0 then Invalid; return 0; end if;
            Gamma := Deg (Atan2 ((Args (2) / Args (3)) / (2.0 * Pi * A), 1.0));
            Bank := Barrel_Bank (R, E.Speed); R2 := R / (1.0 - Cos (Rad (90.0 - Gamma)));
            Horizontal (-R2, 90.0 - Gamma, -Bank);
            Add_Piece ((Kind => Cylinder_G, Radius => R, Distance => Args (2), Spirals => Args (3),
                        Gamma => Atan2 ((Args (2) / Args (3)) / (2.0 * Pi * A), 1.0), others => <>),
                       (others => <>));
            Horizontal (R2, 90.0 - Gamma, Bank);
         when Crossbox_Tophat =>
            Vertical (R, 90.0); Sequence_Straight (Args (2) - 2.0 * R, 90.0); Set_Orient (0.0, 90.0, 90.0);
            Vertical (-R, 90.0); Set_Orient (180.0, 0.0, 90.0);
            Straight (Args (3) - 2.0 * R, 0.0); Vertical (R, 90.0); Set_Orient (0.0, -90.0, 90.0);
            Sequence_Straight (Args (2) - 2.0 * R, -90.0); Set_Orient (0.0, -90.0, 180.0);
            Vertical (R, 90.0); Set_Orient (0.0, 0.0, 180.0);
         when Triangular_Loop =>
            H3 := Args (2) - 2.0 * R; Side := H3 / Cos (Rad (45.0)); Base := 2.0 * (H3 + R) - 2.0 * R;
            Straight (Base * 1.0 / 5.0, 180.0); Straight (Base * 2.0 / 5.0, 0.0); Vertical (R, 135.0);
            for I in 1 .. 2 loop
               Corner := (if I = 1 then 90.0 else 135.0);
               Straight (Side * 2.0 / 9.0, 0.0); Straight (Side * 2.0 / 9.0, 90.0);
               Straight (Side * 1.0 / 9.0, 0.0); Straight (Side * 2.0 / 9.0, 90.0);
               Straight (Side * 2.0 / 9.0, 0.0); Vertical (R, Corner);
            end loop;
            Straight (Base * 2.0 / 5.0, 0.0); Straight (Base * 1.0 / 5.0, 180.0); Straight (Base * 2.0 / 5.0, 0.0);
         when Unknown => Invalid; return 0;
      end case;
      N := Compose (First, Last, Width, E);
      if N /= 0 and then Cmd = Barrel_Roll then
         Nodes (N).Speed_Dependent := True; Nodes (N).Bank_Radius := R;
      end if;
      return N;
   end Build;

   type Configuration is record
      Ang_Accel, Ang_Tc, Thr_Pit_Ff, Spd_P, Spd_I, Roll_Corr_Tc, Time_Corr_P,
      Err_Corr_P, Err_Corr_D, Entry_Rate, Thr_Lookahead, Thr_Min, Thr_Boost, Yaw_Accel,
      Lookahead, Path_Scale, Ke_Rudd, Ke_Rudd_Lookahead, Alt_Abort,
      Airspeed_Min, Scaling_Speed, Airspeed_Cruise : F := 0.0;
   end record;
   type Input is record
      Now : F := 0.0;
      Pos, Gyro, Velocity : V := Zero;
      Airspeed, Eas2tas : F := 0.0;
      Attitude : Q := Identity;
      Throttle : F := 0.0;
   end record;
   procedure Validate (P : Configuration) is
      Values : constant Floats :=
        (P.Ang_Accel, P.Ang_Tc, P.Thr_Pit_Ff, P.Spd_P, P.Spd_I, P.Roll_Corr_Tc,
         P.Time_Corr_P, P.Err_Corr_P, P.Err_Corr_D, P.Entry_Rate, P.Thr_Lookahead,
         P.Thr_Min, P.Thr_Boost, P.Yaw_Accel, P.Lookahead, P.Path_Scale, P.Ke_Rudd,
         P.Ke_Rudd_Lookahead, P.Alt_Abort, P.Airspeed_Min, P.Scaling_Speed, P.Airspeed_Cruise);
   begin
      for Value of Values loop if not Finite (Value) then Invalid; return; end if; end loop;
      if P.Roll_Corr_Tc <= 0.0 or else P.Ang_Tc < 0.0 or else P.Entry_Rate <= 0.0 or else P.Path_Scale = 0.0
        or else P.Airspeed_Min <= 0.0 or else P.Scaling_Speed <= 0.0 or else P.Airspeed_Cruise <= 0.0
        or else P.Thr_Min < 0.0 or else P.Thr_Min > 100.0 or else P.Thr_Boost < 0.0 or else P.Thr_Boost > 100.0
      then Invalid; end if;
   end Validate;
   procedure Validate (X : Input) is
      A : Q renames X.Attitude;
      Norm : constant F := A.Q1 * A.Q1 + A.Q2 * A.Q2 + A.Q3 * A.Q3 + A.Q4 * A.Q4;
   begin
      if not Finite (X.Now) or else not Finite (X.Pos) or else not Finite (X.Gyro) or else not Finite (X.Velocity)
        or else not Finite (X.Airspeed) or else X.Airspeed < 0.0 or else not Finite (X.Eas2tas)
        or else X.Eas2tas <= 0.0 or else not Finite (X.Throttle) or else not Finite (Norm)
        or else abs (Norm - 1.0) > 0.01 then Invalid; end if;
   end Validate;
   function Target_Speed (X : Input; P : Configuration) return F is
     (Max (X.Eas2tas * P.Airspeed_Cruise, Length (X.Velocity)));
   type Output is record
      Throttle : F := 0.0;
      Rates : V := Zero;
      Rudder, Position_Error : F := 0.0;
      Path_Rate_Fallback, Lookahead_Fallback : Boolean := False;
      Track_Pos : V := Zero;
      Track_Attitude, Measured_Attitude : Q := Identity;
      Path_T, Path_Error_T, Q_Change_T, Dt : F := 0.0;
      Correction_Rates, Path_Rates : V := Zero;
      Roll_Error_Rate, Angle_Error : F := 0.0;
      Have_Lookahead : Boolean := False;
      Lookahead_Rates : Args_Type := (others => 0.0);
      Speed_Log : Floats (1 .. 6) := (others => 0.0);
   end record;
   type Controller is record
      Count : Unsigned_32 := 0;
      Target_Speed, Length, Total_Time : F := 0.0;
      Initial_Orientation, Accumulated_Orientation : Q := Identity;
      Initial_Pos, Last_Rates, Pos, Shift, Tangent : V := Zero;
      Last_Time, Path_T, Roll : F := 0.0;
      Last_Shift_Xy : Boolean := False;
      Spd_P, Spd_I, Integral, Total, Spd_Last_Time : F := 0.0;
   end record;
   pragma Suppress_Initialization (Controller);
   State : Controller;
   procedure Save_Controller (W : out Controller_Words) is
   begin
      W (0) := State.Count;
      W (1) := Bits (State.Target_Speed);
      W (2) := Bits (State.Length);
      W (3) := Bits (State.Total_Time);
      W (4) := Bits (State.Initial_Orientation.Q1);
      W (5) := Bits (State.Initial_Orientation.Q2);
      W (6) := Bits (State.Initial_Orientation.Q3);
      W (7) := Bits (State.Initial_Orientation.Q4);
      W (8) := Bits (State.Initial_Pos.X);
      W (9) := Bits (State.Initial_Pos.Y);
      W (10) := Bits (State.Initial_Pos.Z);
      W (11) := Bits (State.Accumulated_Orientation.Q1);
      W (12) := Bits (State.Accumulated_Orientation.Q2);
      W (13) := Bits (State.Accumulated_Orientation.Q3);
      W (14) := Bits (State.Accumulated_Orientation.Q4);
      W (15) := Bits (State.Last_Time);
      W (16) := Bits (State.Last_Rates.X);
      W (17) := Bits (State.Last_Rates.Y);
      W (18) := Bits (State.Last_Rates.Z);
      W (19) := Bits (State.Path_T);
      W (20) := Bits (State.Pos.X);
      W (21) := Bits (State.Pos.Y);
      W (22) := Bits (State.Pos.Z);
      W (23) := Bits (State.Roll);
      W (24) := Bits (State.Shift.X);
      W (25) := Bits (State.Shift.Y);
      W (26) := Bits (State.Shift.Z);
      W (27) := (if State.Last_Shift_Xy then 1 else 0);
      W (28) := Bits (State.Tangent.X);
      W (29) := Bits (State.Tangent.Y);
      W (30) := Bits (State.Tangent.Z);
      W (31) := Bits (State.Spd_P);
      W (32) := Bits (State.Spd_I);
      W (33) := Bits (State.Integral);
      W (34) := Bits (State.Total);
      W (35) := Bits (State.Spd_Last_Time);
   end Save_Controller;
   procedure Restore_Controller (W : Controller_Words) is
      Norm : F;
   begin
      for I in 1 .. 35 loop
         if I /= 27 and then not Finite (From_Bits (W (I))) then Invalid; return; end if;
      end loop;
      if W (27) > 1 then Invalid; return; end if;
      State.Count := W (0);
      State.Target_Speed := From_Bits (W (1));
      State.Length := From_Bits (W (2));
      State.Total_Time := From_Bits (W (3));
      State.Initial_Orientation.Q1 := From_Bits (W (4));
      State.Initial_Orientation.Q2 := From_Bits (W (5));
      State.Initial_Orientation.Q3 := From_Bits (W (6));
      State.Initial_Orientation.Q4 := From_Bits (W (7));
      State.Initial_Pos.X := From_Bits (W (8));
      State.Initial_Pos.Y := From_Bits (W (9));
      State.Initial_Pos.Z := From_Bits (W (10));
      State.Accumulated_Orientation.Q1 := From_Bits (W (11));
      State.Accumulated_Orientation.Q2 := From_Bits (W (12));
      State.Accumulated_Orientation.Q3 := From_Bits (W (13));
      State.Accumulated_Orientation.Q4 := From_Bits (W (14));
      State.Last_Time := From_Bits (W (15));
      State.Last_Rates.X := From_Bits (W (16));
      State.Last_Rates.Y := From_Bits (W (17));
      State.Last_Rates.Z := From_Bits (W (18));
      State.Path_T := From_Bits (W (19));
      State.Pos.X := From_Bits (W (20));
      State.Pos.Y := From_Bits (W (21));
      State.Pos.Z := From_Bits (W (22));
      State.Roll := From_Bits (W (23));
      State.Shift.X := From_Bits (W (24));
      State.Shift.Y := From_Bits (W (25));
      State.Shift.Z := From_Bits (W (26));
      State.Last_Shift_Xy := W (27) /= 0;
      State.Tangent.X := From_Bits (W (28));
      State.Tangent.Y := From_Bits (W (29));
      State.Tangent.Z := From_Bits (W (30));
      State.Spd_P := From_Bits (W (31));
      State.Spd_I := From_Bits (W (32));
      State.Integral := From_Bits (W (33));
      State.Total := From_Bits (W (34));
      State.Spd_Last_Time := From_Bits (W (35));
      if State.Target_Speed <= 0.0 or else State.Length <= 0.0 or else State.Total_Time <= 0.0
      then Invalid; return; end if;
      Norm := State.Initial_Orientation.Q1 ** 2 + State.Initial_Orientation.Q2 ** 2
        + State.Initial_Orientation.Q3 ** 2 + State.Initial_Orientation.Q4 ** 2;
      if abs (Norm - 1.0) > 0.01 then Invalid; return; end if;
      Norm := State.Accumulated_Orientation.Q1 ** 2 + State.Accumulated_Orientation.Q2 ** 2
        + State.Accumulated_Orientation.Q3 ** 2 + State.Accumulated_Orientation.Q4 ** 2;
      if abs (Norm - 1.0) > 0.01 then Invalid; end if;
   end Restore_Controller;
   procedure Increment (Value : in out Unsigned_32) is
   begin
      if Value /= Unsigned_32'Last then Value := Value + 1; end if;
   end Increment;
   procedure Sample (N : Node_Id; Time : F; X : Input; P : Configuration;
                     Point : out V; Angle : out F; Attr : out Attributes) is
      Local_T : constant F := Constrain (Time, 0.0, 1.0);
      E : constant Evaluation := (Target_Speed (X, P), P.Entry_Rate, True);
      Roll, Corr : F;
      Inv : Q;
   begin
      Point := Path_Position (N, Local_T, E);
      Roll := Path_Roll (N, Local_T, State.Total_Time, E);
      Corr := Path_Correction (N, Local_T, E);
      Attr := Path_Attributes (N, Local_T, E);
      Point := Scale (Earth_To_Body (State.Initial_Orientation, Add (Point, State.Shift)), abs P.Path_Scale);
      if P.Path_Scale < 0.0 then
         Point.Y := -Point.Y; Corr := -Corr; Roll := -Roll;
         Inv := Inverse (State.Initial_Orientation);
         Point := Body_To_Earth (Mul (Inv, Inv), Point);
      end if;
      Point := Add (Point, State.Initial_Pos); Angle := Rad (Roll + Corr);
   end Sample;
   procedure Start_Controller (N : Node_Id; X : Input; P : Configuration; Yaw : F; Count : Unsigned_32) is
      Speed, Path_Length, Unused_Angle : F;
      Rot : Q;
      Point : V;
      Attr : Attributes;
   begin
      Validate (P); Validate (X);
      if not Finite (Yaw) then Invalid; end if;
      if Fault /= OK then return; end if;
      Speed := Target_Speed (X, P); Path_Length := Nodes (N).Length * abs P.Path_Scale;
      if not Finite (Speed) or else Speed <= 0.0 or else not Finite (Path_Length) or else Path_Length <= 0.0
      then Invalid; return; end if;
      Rot := Orient (0.0, 0.0, Yaw); Normalize (Rot);
      if Nodes (N).Kind = Composite_Path then Nodes (N).Highest := 0; end if;
      State := (Count => Count, Target_Speed => Speed, Length => Path_Length, Total_Time => Path_Length / Speed,
                Initial_Orientation => Rot, Initial_Pos => X.Pos, Accumulated_Orientation => Rot,
                Last_Time => X.Now - 1.0 / 40.0, Last_Rates => Scale (X.Gyro, Deg (1.0)), Pos => X.Pos,
                Spd_P => P.Spd_P, Spd_I => P.Spd_I, Integral => X.Throttle, Spd_Last_Time => X.Now, others => <>);
      Increment (State.Count);
      Sample (N, 0.1 / (State.Total_Time * 40.0), X, P, Point, Unused_Angle, Attr);
      State.Tangent := Sub (Point, State.Pos);
   end Start_Controller;
   type Step_Kind is (Waiting, Command_Step, Altitude_Abort, Finished);
   procedure Controller_Step (N : Node_Id; X : Input; P : Configuration; Kind : out Step_Kind; O : out Output) is
      Attitude, Delta_Q, Rolled, Desired, Roll_Error, Anticipated, Change : Q;
      Velocity_Length, Dt, Local_Dt, Airspeed, R0, R1, Unused_Roll, Path_Distance, Path_Delta,
      Last_Path_T, Path_Distance_Error, Path_Error_T, Q_Change_T, Alpha, Tas, Angle,
      Ratio, Max_Delta, Max_Yaw, Rudder, Scale_Factor, Pitch, Spd_Dt, Error, Proportional,
      Feedforward, Minimum, Throttle : F;
      P0, P1, P2, Current, Path_Point, Shift, Tangent1, Tangent2, Tangent3, Tv, Position_Error,
      Acceleration, Acc_Body, Correction_Rate, Path_Rate, Rate_Earth, Rate_Body, Axis,
      Axis_Angle_Vector, Error_Axis, Error_Rate, Error_Body, Rate, Ahead, Total, Difference, Rates : V;
      Attr, Ahead_Attr : Attributes;
      Shift_Xy, Path_Fallback : Boolean;
   begin
      Kind := Finished; O := (others => <>);
      Validate (P); Validate (X);
      if Fault /= OK then return; end if;
      Increment (State.Count);
      Attitude := Inverse (X.Attitude); Velocity_Length := Length (X.Velocity);
      Dt := X.Now - State.Last_Time;
      if Dt < 0.0 then Invalid; return; end if;
      if Dt < 0.25 / 40.0 then Kind := Waiting; return; end if;
      if not Finite (Dt) then Invalid; return; end if;
      State.Last_Time := X.Now; Local_Dt := (1.0 / 40.0) / State.Total_Time;
      if State.Path_T + Local_Dt > 1.0 then return; end if;
      Airspeed := Max (P.Airspeed_Min, X.Airspeed); P0 := State.Pos; R0 := State.Roll;
      Sample (N, State.Path_T + Local_Dt, X, P, P1, Unused_Roll, Attr);
      if Fault /= OK then return; end if;
      Shift_Xy := Attr.Shift_Xy.Present and then Attr.Shift_Xy.Value;
      if Shift_Xy and then not State.Last_Shift_Xy then
         Current := Body_To_Earth (State.Initial_Orientation, X.Pos);
         Path_Point := Body_To_Earth (State.Initial_Orientation, P1);
         Shift := Sub (Current, Path_Point); Shift.Z := 0.0;
         State.Shift := Add (State.Shift, Shift); P1 := Add (P1, Earth_To_Body (State.Initial_Orientation, Shift));
         P0.X := P1.X; P0.Y := P1.Y;
      end if;
      State.Last_Shift_Xy := Shift_Xy; Tangent1 := State.Tangent;
      Tangent2 := Sub (P1, P0); Tv := Unit (Tangent2);
      Path_Distance := Dot (X.Velocity, Tv) * Dt;
      Path_Delta := Constrain (Path_Distance / State.Length, 0.2 * Local_Dt, 4.0 * Local_Dt);
      Sample (N, Constrain (State.Path_T + Path_Delta, 0.0, 1.0), X, P, P1, Unused_Roll, Attr);
      if Fault /= OK then return; end if;
      Last_Path_T := State.Path_T; State.Path_T := State.Path_T + Path_Delta;
      Tangent2 := Sub (P1, P0); Tv := Unit (Tangent2);
      Position_Error := Sub (X.Pos, P1); Path_Distance_Error := Dot (Tv, Position_Error);
      Path_Error_T := Constrain (Path_Distance_Error / State.Length, -0.9 * Path_Delta, 2.0 * Path_Delta);
      State.Path_T := State.Path_T + P.Time_Corr_P * Path_Error_T;
      Sample (N, Constrain (State.Path_T, 0.0, 1.0), X, P, P1, R1, Attr);
      if Fault /= OK then return; end if;
      Tangent2 := Sub (P1, P0); Q_Change_T := (State.Path_T - Last_Path_T) * State.Total_Time;
      R1 := State.Roll + Wrap_Pi (R1 - State.Roll);
      Alpha := Q_Change_T / (Q_Change_T + P.Ang_Tc / (Pi * 2.0));
      R1 := (1.0 - Alpha) * State.Roll + Alpha * R1; R1 := Wrap_Pi (R1);
      State.Tangent := Tangent2; State.Pos := P1; State.Roll := R1;
      Acceleration := Add (Scale (Ortho (Position_Error, Tv), P.Err_Corr_P),
                           Scale (Ortho (X.Velocity, Tv), P.Err_Corr_D));
      if Attr.Pos_Gain_Mul.Present then Acceleration := Scale (Acceleration, Attr.Pos_Gain_Mul.Value); end if;
      Acceleration := Scale (Acceleration, (if Attr.Pos_Corr.Present then Attr.Pos_Corr.Value else 1.0));
      Acc_Body := Earth_To_Body (Attitude, Acceleration); Tas := Constrain (X.Eas2tas * Airspeed, 3.0, 100.0);
      Correction_Rate := (0.0, -Acc_Body.Z / Tas, Acc_Body.Y / Tas);
      if Is_Nan (Correction_Rate) then Correction_Rate := Zero; end if;
      Correction_Rate := Scale (Correction_Rate, Deg (1.0));
      if State.Count < 2 then Correction_Rate := Zero; end if;
      if Fault /= OK then return; end if;
      Path_Rate := Tangents_To_Rate (Tangent1, Tangent2, Dt); Path_Fallback := False;
      if Fault = S.INVALID or else (Fault = OK and then not Finite (Path_Rate)) then
         Fault := OK; Path_Rate := Zero; Path_Fallback := True;
      end if;
      if Fault /= OK then return; end if;
      Rate_Earth := Scale (Path_Rate, Deg (1.0));
      if State.Count < 3 then Rate_Earth.Z := 0.0; end if;
      Rate_Body := Earth_To_Body (Attitude, Rate_Earth); Rate_Body.X := Deg (Wrap_Pi (R1 - R0) / Dt);
      Delta_Q := Angular_Velocity (Path_Rate, Dt);
      State.Accumulated_Orientation := Mul (Delta_Q, State.Accumulated_Orientation); Normalize (State.Accumulated_Orientation);
      Axis := Earth_To_Body (State.Accumulated_Orientation, (1.0, 0.0, 0.0));
      Rolled := Axis_Angle (Axis, R1); Desired := Mul (Rolled, State.Accumulated_Orientation);
      Roll_Error := Mul (Desired, Attitude); Normalize (Roll_Error);
      Axis_Angle_Vector := To_Axis_Angle (Roll_Error); Angle := Length (Axis_Angle_Vector);
      if Fault /= OK then return; end if;
      if Angle < 0.00001 then Angle := 0.0; Error_Axis := (1.0, 0.0, 0.0);
      else Error_Axis := Scale (Axis_Angle_Vector, 1.0 / Angle); end if;
      Error_Rate := Scale (Error_Axis, Angle / P.Roll_Corr_Tc);
      Error_Body := Scale (Earth_To_Body (Attitude, Error_Rate), Deg (1.0)); Error_Body.Y := 0.0; Error_Body.Z := 0.0;
      if P.Lookahead > 0.0 then
         Sample (N, Constrain (State.Path_T + P.Lookahead / State.Total_Time, 0.0, 1.0), X, P, P2, Unused_Roll, Ahead_Attr);
         if Fault /= OK then return; end if;
         Tangent3 := Sub (P2, P1); Rate := Tangents_To_Rate (Tangent2, Tangent3, 0.5 * (P.Lookahead + 1.0 / 40.0));
         if Fault = S.INVALID then Fault := OK; O.Lookahead_Fallback := True;
         elsif Fault = OK then
            Ratio := Velocity_Length / State.Target_Speed; Rate := Scale (Rate, Ratio * Ratio);
            Ahead := Scale (Earth_To_Body (Attitude, Rate), Deg (1.0)); O.Have_Lookahead := True;
            O.Lookahead_Rates := (Rate_Body.Y, Ahead.Y, Rate_Body.Z, Ahead.Z);
            if Finite (Ahead) then Rate_Body.Y := Ahead.Y; Rate_Body.Z := Ahead.Z;
            else O.Lookahead_Fallback := True; end if;
         end if;
      end if;
      if Fault /= OK then return; end if;
      Total := Add (Add (Add (Rate_Body, Correction_Rate), Error_Body), Zero);
      Difference := Sub (Total, State.Last_Rates); Max_Delta := P.Ang_Accel * Dt; Max_Yaw := Max_Delta;
      if P.Yaw_Accel > 0.0 and then (P.Yaw_Accel < P.Ang_Accel or else P.Ang_Accel <= 0.0) then Max_Yaw := P.Yaw_Accel * Dt; end if;
      if Max_Delta > 0.0 then
         Difference.X := Constrain (Difference.X, -Max_Delta, Max_Delta);
         Difference.Y := Constrain (Difference.Y, -Max_Delta, Max_Delta);
      end if;
      if Max_Yaw > 0.0 then Difference.Z := Constrain (Difference.Z, -Max_Yaw, Max_Yaw); end if;
      Rates := Add (State.Last_Rates, Difference); State.Last_Rates := Rates; Rudder := 0.0;
      if P.Ke_Rudd > 0.0 then
         Change := Angular_Velocity (X.Gyro, -P.Ke_Rudd_Lookahead); Anticipated := Inverse (Mul (Change, Attitude));
         Scale_Factor := P.Scaling_Speed / Airspeed;
         Rudder := Constrain (-2.0 * (Anticipated.Q3 * Anticipated.Q4 + Anticipated.Q1 * Anticipated.Q2)
                             * P.Ke_Rudd * (Scale_Factor * Scale_Factor), -100.0, 100.0);
      end if;
      Change := Angular_Velocity (Path_Rate, P.Thr_Lookahead); Anticipated := Mul (Change, Desired);
      Pitch := Max (Euler_Pitch (Anticipated), Euler_Pitch (Desired));
      Spd_Dt := X.Now - State.Spd_Last_Time; State.Spd_Last_Time := X.Now;
      Error := State.Target_Speed - Velocity_Length; Proportional := State.Spd_P * Error;
      if (State.Total < 100.0 and then State.Total > 0.0) or else (State.Total >= 100.0 and then Error < 0.0)
        or else (State.Total <= 0.0 and then Error > 0.0)
      then State.Integral := State.Integral + State.Spd_I * Error * Spd_Dt; end if;
      State.Integral := Constrain (State.Integral, -100.0, 100.0);
      State.Total := Constrain (Proportional + State.Integral, 0.0, 100.0); Feedforward := Sin (Pitch) * P.Thr_Pit_Ff;
      Minimum := P.Thr_Min;
      if Attr.Thr_Boost.Present and then Attr.Thr_Boost.Value then Minimum := Max (Minimum, P.Thr_Boost); end if;
      Throttle := Constrain (State.Total + Feedforward, Minimum, 100.0);
      if not Finite (Throttle) or else not Finite (Rates) or else not Finite (Rudder) then Invalid; return; end if;
      O.Throttle := Throttle; O.Rates := Rates; O.Rudder := Rudder; O.Position_Error := Length (Position_Error);
      O.Path_Rate_Fallback := Path_Fallback; O.Track_Pos := P1; O.Track_Attitude := Desired;
      O.Measured_Attitude := X.Attitude; O.Path_T := State.Path_T; O.Path_Error_T := Path_Error_T;
      O.Q_Change_T := Q_Change_T; O.Dt := Dt; O.Correction_Rates := Correction_Rate; O.Path_Rates := Rate_Body;
      O.Roll_Error_Rate := Error_Body.X; O.Angle_Error := Wrap_180 (Deg (Angle));
      O.Speed_Log := (State.Target_Speed, Velocity_Length, Proportional, State.Integral, State.Total, Feedforward);
      Kind := (if Sub (X.Pos, State.Initial_Pos).Z > P.Alt_Abort then Altitude_Abort else Command_Step);
   end Controller_Step;
   -- Generated APIs express Busy as a status. Retry reads for at most 100 ms.
   function Retry_Again (Code : Status; Start : Unsigned_64) return Boolean is
      Sleep_Code : Status;
   begin
      if Code = S.BUSY and then Elapsed (Time_Us, Start) < 100_000 then
         Sleep_Code := Sleep_Ms (Ctx, 1);
         if Sleep_Code /= OK then Check (Sleep_Code); return False; end if;
         return True;
      end if;
      Check (Code); return False;
   end Retry_Again;
   function Read_Parameter (Name : String) return F is
      Value : T.Optional_Float;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
   begin
      if Fault /= OK then return 0.0; end if;
      loop
         Code := B.Param.Get (Ctx, Name, Value);
         exit when not Retry_Again (Code, Start);
      end loop;
      if Fault = OK and then (not Boolean (Value.Present) or else not Finite (Value.Value)) then Invalid; end if;
      return (if Fault = OK then Value.Value else 0.0);
   end Read_Parameter;
   type Param_Definition is record
      Index : Unsigned_8;
      Length : Positive;
      Name : String (1 .. 10);
      Default : F;
   end record;
   Aerom : constant array (Positive range <>) of Param_Definition :=
     ((1, 9, "ANG_ACCEL ", 6000.0), (2, 6, "ANG_TC    ", 0.1),
      (4, 10, "THR_PIT_FF", 80.0), (5, 5, "SPD_P     ", 5.0), (6, 5, "SPD_I     ", 25.0),
      (8, 10, "ROL_COR_TC", 0.25), (11, 10, "TIME_COR_P", 1.0), (12, 9, "ERR_COR_P ", 2.0),
      (13, 9, "ERR_COR_D ", 2.8), (14, 10, "ENTRY_RATE", 60.0), (15, 9, "THR_LKAHD ", 1.0),
      (16, 5, "DEBUG     ", 0.0), (17, 7, "THR_MIN   ", 0.0), (18, 9, "THR_BOOST ", 50.0),
      (19, 9, "YAW_ACCEL ", 1500.0), (20, 5, "LKAHD     ", 0.5), (21, 10, "PATH_SCALE", 1.0),
      (22, 9, "BOX_WIDTH ", 400.0), (23, 9, "STALL_THR ", 40.0), (24, 9, "STALL_PIT ", -20.0),
      (26, 7, "KE_RUDD   ", 25.0), (27, 10, "KE_RUDD_LK", 0.25), (28, 9, "ALT_ABORT ", 15.0),
      (29, 4, "TS_P      ", 0.33), (30, 4, "TS_I      ", 0.33), (31, 9, "TS_SPDMAX ", 0.0),
      (32, 7, "TS_RATE   ", 4.0), (33, 9, "MIS_ANGLE ", 0.0), (34, 7, "OPTIONS   ", 0.0),
      (35, 4, "LANG      ", 0.0));
   type Param_Name is record
      Data : String (1 .. 16) := (others => ' ');
      Length : Natural range 0 .. 16 := 0;
   end record;
   pragma Suppress_Initialization (Param_Name);
   Aerom_Names : array (0 .. 40) of Param_Name;
   Trik_Names : array (0 .. 63) of Param_Name;
   Trick_Count : Natural range 0 .. 11 := 0;
   Startup_Spd_P, Startup_Spd_I : F := 0.0;
   procedure Add_Table (Key : Unsigned_8; Prefix : String; Size : Unsigned_8) is
      Value : Interfaces.C.C_bool;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
   begin
      if Fault /= OK then return; end if;
      loop
         Code := B.Param.Add_Table (Ctx, Key, Prefix, Size, Value);
         exit when not Retry_Again (Code, Start);
      end loop;
      if Fault = OK and then not Boolean (Value) then Check (S.HOST_ERROR); end if;
   end Add_Table;
   procedure Add_Param (Key, Index : Unsigned_8; Prefix, Name : String; Default : F; Target : out Param_Name) is
      Value : Interfaces.C.C_bool;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
      Ignored : F;
   begin
      Target := (others => <>);
      if Fault /= OK then return; end if;
      loop
         Code := B.Param.Add_Param (Ctx, Key, Index, Name, Default, Value);
         exit when not Retry_Again (Code, Start);
      end loop;
      if Fault = OK and then not Boolean (Value) then Check (S.HOST_ERROR); end if;
      if Prefix'Length + Name'Length > 16 then Invalid; return; end if;
      Target.Length := Prefix'Length + Name'Length;
      Target.Data (1 .. Prefix'Length) := Prefix;
      Target.Data (Prefix'Length + 1 .. Target.Length) := Name;
      Ignored := Read_Parameter (Target.Data (1 .. Target.Length));
   end Add_Param;
   function A (Index : Natural) return F is
   begin
      if Index > Aerom_Names'Last or else Aerom_Names (Index).Length = 0 then Invalid; return 0.0; end if;
      return Read_Parameter (Aerom_Names (Index).Data (1 .. Aerom_Names (Index).Length));
   end A;
   function Trik (Index : Natural) return F is
   begin
      if Index > Trik_Names'Last or else Trik_Names (Index).Length = 0 then Invalid; return 0.0; end if;
      return Read_Parameter (Trik_Names (Index).Data (1 .. Trik_Names (Index).Length));
   end Trik;
   procedure Parameters_Init is
      Code : Status;
      Value : Interfaces.C.C_bool;
      Accel, Ignored : F;
   begin
      Add_Table (70, "AEROM_", 40); Add_Table (71, "TRIK", 63);
      for P of Aerom loop
         Add_Param (70, P.Index, "AEROM_", P.Name (1 .. P.Length), P.Default, Aerom_Names (Natural (P.Index)));
      end loop;
      Add_Param (71, 1, "TRIK", "_ENABLE", 0.0, Trik_Names (1));
      Startup_Spd_P := A (5); Startup_Spd_I := A (6);
      Ignored := Read_Parameter ("AIRSPEED_MIN");
      Ignored := Read_Parameter ("SCALING_SPEED");
      Ignored := Read_Parameter ("AIRSPEED_CRUISE");
      Accel := A (1);
      if Fault = OK and then Accel > 0.0 and then Accel < 100.0 then
         Code := B.Param.Set_And_Save (Ctx, "AEROM_ANG_ACCEL", 3000.0, Value);
         Check (Code);
      end if;
      if A (2) > 1.0 and then Fault = OK then
         Code := B.Param.Set_And_Save (Ctx, "AEROM_ANG_TC", 0.2, Value);
         Check (Code);
      end if;
      if Trik (1) > 0.0 and then Fault = OK then
         Add_Param (71, 2, "TRIK", "_SEL_FN", 301.0, Trik_Names (2));
         Add_Param (71, 3, "TRIK", "_ACT_FN", 300.0, Trik_Names (3));
         Add_Param (71, 4, "TRIK", "_COUNT", 3.0, Trik_Names (4));
         Trick_Count := Natural (Floor (Constrain (Trik (4), 1.0, 11.0)));
         for I in 1 .. Trick_Count loop
            declare
               Num : constant Number_Text := Decimal (I);
               Prefix : constant String := Num.Data (Num.First .. 10) & "_";
            begin
               Add_Param (71, Unsigned_8 (5 * I), "TRIK", Prefix & "ID", -1.0, Trik_Names (5 * I));
               for J in 1 .. 4 loop
                  declare
                     Digit : constant String (1 .. 1) := (1 => Character'Val (Character'Pos ('0') + J));
                  begin
                     Add_Param (71, Unsigned_8 (5 * I + J), "TRIK", Prefix & "ARG" & Digit,
                                (if J = 1 then 30.0 else 0.0), Trik_Names (5 * I + J));
                  end;
               end loop;
            end;
         end loop;
      end if;
   end Parameters_Init;
   function Read_Configuration return Configuration is
      P : Configuration;
   begin
      P.Ang_Accel := A (1); P.Ang_Tc := A (2); P.Thr_Pit_Ff := A (4);
      P.Spd_P := Startup_Spd_P; P.Spd_I := Startup_Spd_I; P.Roll_Corr_Tc := A (8);
      P.Time_Corr_P := A (11); P.Err_Corr_P := A (12); P.Err_Corr_D := A (13); P.Entry_Rate := A (14);
      P.Thr_Lookahead := A (15); P.Thr_Min := A (17); P.Thr_Boost := A (18); P.Yaw_Accel := A (19);
      P.Lookahead := A (20); P.Path_Scale := A (21); P.Ke_Rudd := A (26); P.Ke_Rudd_Lookahead := A (27);
      P.Alt_Abort := A (28); P.Airspeed_Min := Read_Parameter ("AIRSPEED_MIN");
      P.Scaling_Speed := Read_Parameter ("SCALING_SPEED"); P.Airspeed_Cruise := Read_Parameter ("AIRSPEED_CRUISE");
      return P;
   end Read_Configuration;
   function Observation return Input is
      X : Input;
      Start : constant Unsigned_64 := Time_Us;
      Read_Start : Unsigned_64;
      Code : Status;
      Vec : T.Optional_Vector3f;
      Number : T.Optional_Float;
      Rot : T.Optional_Quaternion;
   begin
      if Fault /= OK then return X; end if;
      X.Now := F (Unsigned_32 ((Time_Us / 1000) mod 2 ** 32)) * 0.001;
      Read_Start := Time_Us;
      loop
         Code := B.Ahrs.Get_Relative_Position_NED_Origin (Ctx, Vec);
         exit when not Retry_Again (Code, Read_Start);
      end loop;
      if Fault = OK and then not Boolean (Vec.Present) then Check (S.UNAVAILABLE); end if;
      if Fault /= OK then return X; end if;
      X.Pos := Vec.Value; Read_Start := Time_Us;
      loop
         Code := B.Ahrs.Get_Gyro (Ctx, X.Gyro);
         exit when not Retry_Again (Code, Read_Start);
      end loop;
      Read_Start := Time_Us;
      loop
         exit when Fault /= OK;
         Code := B.Ahrs.Get_Velocity_NED (Ctx, Vec);
         exit when not Retry_Again (Code, Read_Start);
      end loop;
      if Fault = OK and then not Boolean (Vec.Present) then Check (S.UNAVAILABLE); end if;
      if Fault /= OK then return X; end if;
      X.Velocity := Vec.Value; Read_Start := Time_Us;
      loop
         Code := B.Ahrs.Airspeed_EAS (Ctx, Number);
         exit when not Retry_Again (Code, Read_Start);
      end loop;
      if Fault = OK and then not Boolean (Number.Present) then Check (S.UNAVAILABLE); end if;
      if Fault /= OK then return X; end if;
      X.Airspeed := Number.Value; Read_Start := Time_Us;
      loop
         Code := B.Ahrs.Get_EAS2TAS (Ctx, X.Eas2tas);
         exit when not Retry_Again (Code, Read_Start);
      end loop;
      Read_Start := Time_Us;
      loop
         exit when Fault /= OK;
         Code := B.Ahrs.Get_Quaternion (Ctx, Rot);
         exit when not Retry_Again (Code, Read_Start);
      end loop;
      if Fault = OK and then not Boolean (Rot.Present) then Check (S.UNAVAILABLE); end if;
      if Fault /= OK then return X; end if;
      X.Attitude := Rot.Value; Read_Start := Time_Us;
      loop
         Code := B.Srv_Channels.Get_Output_Scaled (Ctx, T.SrvChannelFunction (70), X.Throttle);
         exit when not Retry_Again (Code, Read_Start);
      end loop;
      if Elapsed (Time_Us, Start) > 10_000 then Check (S.BUSY); end if;
      Validate (X); return X;
   end Observation;

   Max_Bytes : constant := 16384;
   Schedule_Text : String (1 .. Max_Bytes);
   Schedule_Length : Natural range 0 .. Max_Bytes := 0;
   Loaded_Name : String (1 .. 48) := (others => ' ');
   Loaded_Name_Length : Natural range 0 .. 48 := 0;
   function Byte_At (Index : Positive) return Natural is (Character'Pos (Schedule_Text (Index)));
   procedure Validate_Encoding is
      I : Positive := 1;
      Lead, Second, Count : Natural;
   begin
      while I <= Schedule_Length loop
         Lead := Byte_At (I);
         if Lead = 0 then Invalid; return;
         elsif Lead < 128 then Count := 1;
         elsif Lead in 16#C2# .. 16#DF# then Count := 2;
         elsif Lead in 16#E0# .. 16#EF# then Count := 3;
         elsif Lead in 16#F0# .. 16#F4# then Count := 4;
         else Invalid; return; end if;
         if I + Count - 1 > Schedule_Length then Invalid; return; end if;
         for J in 1 .. Count - 1 loop
            if Byte_At (I + J) not in 16#80# .. 16#BF# then Invalid; return; end if;
         end loop;
         if Count >= 3 then
            Second := Byte_At (I + 1);
            if (Lead = 16#E0# and then Second < 16#A0#) or else (Lead = 16#ED# and then Second > 16#9F#)
              or else (Lead = 16#F0# and then Second < 16#90#) or else (Lead = 16#F4# and then Second > 16#8F#)
            then Invalid; return; end if;
         end if;
         I := I + Count;
      end loop;
   end Validate_Encoding;
   function Space_At (Index : Positive) return Boolean is
      First : Positive := Index;
      Code, Count, Lead : Natural;
   begin
      while First > 1 and then Byte_At (First) in 16#80# .. 16#BF# loop First := First - 1; end loop;
      Lead := Byte_At (First);
      if Lead < 128 then Code := Lead; Count := 1;
      elsif Lead < 16#E0# then Code := Lead mod 32; Count := 2;
      elsif Lead < 16#F0# then Code := Lead mod 16; Count := 3;
      else Code := Lead mod 8; Count := 4; end if;
      for J in 1 .. Count - 1 loop Code := Code * 64 + Byte_At (First + J) mod 64; end loop;
      return Code in 9 .. 13 or else Code = 32 or else Code = 16#85# or else Code = 16#A0#
        or else Code = 16#1680# or else Code in 16#2000# .. 16#200A#
        or else Code in 16#2028# .. 16#2029# or else Code = 16#202F#
        or else Code = 16#205F# or else Code = 16#3000#;
   end Space_At;
   function Alphanumeric (C : Character) return Boolean is
     (C in 'a' .. 'z' or else C in 'A' .. 'Z' or else C in '0' .. '9');
   function Number (Text : String) return F is
      Input_Bytes : T.Array_Uint8_T_256 := (others => 0);
      R : T.String_StrtofOutput;
   begin
      if Fault /= OK then return 0.0; end if;
      if Text'Length = 0 or else Text'Length > 255 then Invalid; return 0.0; end if;
      for I in Text'Range loop
         if not (Text (I) in '0' .. '9' or else Text (I) = '+' or else Text (I) = '-'
                 or else Text (I) = '.' or else Text (I) = 'e' or else Text (I) = 'E')
         then Invalid; return 0.0; end if;
         Input_Bytes (I - Text'First) := Unsigned_8 (Character'Pos (Text (I)));
      end loop;
      Check (B.String.Strtof (Ctx, Input_Bytes, R));
      if Fault = OK and then (Natural (R.Consumed) /= Text'Length or else not Finite (R.Value)) then Invalid; end if;
      return (if Fault = OK then R.Value else 0.0);
   end Number;
   function Bool_Value (Text : String) return Boolean is
   begin
      if Text = "true" then return True;
      elsif Text = "false" then return False;
      else Invalid; return False; end if;
   end Bool_Value;
   procedure Set_Name (Name : String) is
   begin
      Loaded_Name_Length := Name'Length;
      Loaded_Name (1 .. Name'Length) := Name;
   end Set_Name;
   function Parse_Schedule (Width : F; E : Evaluation) return Node_Id is
      First, Last, N : Node_Id := 0;
      Line_First, Line_Last, Start, Finish, Colon, Word_First, Word_Last : Natural;
      Cursor : Natural := 1;
      Count, Arg_Count : Natural := 0;
      Cmd : Command;
      Args : Args_Type;
      Attr : Attributes := No_Attributes;
      Pending : Boolean := False;
      Msg : String (1 .. 50) := (others => ' ');
      Msg_Length : Natural := 0;
   begin
      Set_Name ("Trick");
      Validate_Encoding;
      if Fault /= OK then return 0; end if;
      while Cursor <= Schedule_Length and then Fault = OK loop
         Line_First := Cursor;
         while Cursor <= Schedule_Length and then Schedule_Text (Cursor) /= Character'Val (10) loop Cursor := Cursor + 1; end loop;
         Line_Last := Cursor - 1; Cursor := Cursor + 1;
         while Line_First <= Line_Last and then Space_At (Line_First) loop Line_First := Line_First + 1; end loop;
         while Line_Last >= Line_First and then Space_At (Line_Last) loop Line_Last := Line_Last - 1; end loop;
         if Line_First <= Line_Last and then Schedule_Text (Line_First) /= '#' then
            Colon := Line_First;
            while Colon <= Line_Last and then Schedule_Text (Colon) /= ':' loop Colon := Colon + 1; end loop;
            if Colon <= Line_Last then
               Start := Colon + 1; Finish := Line_Last;
               while Start <= Finish and then Space_At (Start) loop Start := Start + 1; end loop;
               while Finish >= Start and then Space_At (Finish) loop Finish := Finish - 1; end loop;
               declare
                  Key : constant String := Schedule_Text (Line_First .. Colon - 1);
                  Value : constant String := Schedule_Text (Start .. Finish);
               begin
                  if Key = "name" then
                     if Value'Length = 0 or else Value'Length > 48 then Invalid; return 0; end if;
                     for C of Value loop if not Alphanumeric (C) and then C /= '_' then Invalid; return 0; end if; end loop;
                     Set_Name (Value);
                  else
                     if Key = "message" then
                        if Value'Length = 0 or else Value'Length > 50 then Invalid; return 0; end if;
                        for C of Value loop
                           if not Alphanumeric (C) and then C /= '_' and then C /= ':' and then C /= ' '
                             and then C /= '-' and then C /= Character'Val (9) then Invalid; return 0; end if;
                        end loop;
                        Msg_Length := Value'Length; Msg (1 .. Msg_Length) := Value;
                     elsif Key = "roll_ref" then Attr.Roll_Ref := (True, Number (Value));
                     elsif Key = "thr_boost" then Attr.Thr_Boost := (True, Bool_Value (Value));
                     elsif Key = "pos_corr" then Attr.Pos_Corr := (True, Number (Value));
                     elsif Key = "pos_gain_mul" then Attr.Pos_Gain_Mul := (True, Number (Value));
                     elsif Key = "shift_xy" then Attr.Shift_Xy := (True, Bool_Value (Value));
                     else Invalid; return 0; end if;
                     Pending := True;
                  end if;
               end;
            else
               Word_Last := Line_First;
               while Word_Last <= Line_Last and then not Space_At (Word_Last) loop Word_Last := Word_Last + 1; end loop;
               Cmd := Named (Schedule_Text (Line_First .. Word_Last - 1));
               if Cmd = Unknown or else Count = 256 then Invalid; return 0; end if;
               Args := (others => 0.0); Arg_Count := 0;
               Start := Word_Last;
               while Start <= Line_Last and then Fault = OK loop
                  while Start <= Line_Last and then Space_At (Start) loop Start := Start + 1; end loop;
                  exit when Start > Line_Last;
                  Word_First := Start;
                  while Start <= Line_Last and then not Space_At (Start) loop Start := Start + 1; end loop;
                  if Arg_Count = 4 then Invalid; return 0; end if;
                  Arg_Count := Arg_Count + 1; Args (Arg_Count) := Number (Schedule_Text (Word_First .. Start - 1));
               end loop;
               if Fault /= OK then return 0; end if;
               N := Build (Cmd, Args, Width, E);
               if Fault /= OK then return 0; end if;
               Nodes (N).Attr := Attr; Nodes (N).Rebuild := True;
               Nodes (N).Message := Msg; Nodes (N).Message_Length := Msg_Length;
               Append (First, Last, N); Count := Count + 1;
               Attr := No_Attributes; Msg_Length := 0; Pending := False;
            end if;
         end if;
      end loop;
      if Count = 0 or else Pending then Invalid; return 0; end if;
      return Compose (First, Last, Width, E);
   end Parse_Schedule;
   function Read_File (Filename : String) return Boolean is
      Path_Bytes : T.Array_Uint8_T_256 := (others => 0);
      Chunk : T.Optional_File_Read_AtOutput;
      Count : Natural;
   begin
      if Fault /= OK then return False; end if;
      if Filename'Length > 255 then Invalid; return False; end if;
      for I in Filename'Range loop Path_Bytes (I - Filename'First) := Unsigned_8 (Character'Pos (Filename (I))); end loop;
      Schedule_Length := 0;
      loop
         Check (B.File.Read_At (Ctx, Path_Bytes, Unsigned_32 (Schedule_Length), 256, Chunk));
         if Fault /= OK then return False; end if;
         if not Boolean (Chunk.Present) then
            if Schedule_Length /= 0 then Check (S.HOST_ERROR); end if;
            return False;
         end if;
         Count := Natural (Chunk.Value.Length);
         if Count = 0 then return True; end if;
         if Schedule_Length + Count > Max_Bytes then Invalid; return False; end if;
         for I in 0 .. Count - 1 loop Schedule_Text (Schedule_Length + I + 1) := Character'Val (Chunk.Value.Data (I)); end loop;
         Schedule_Length := Schedule_Length + Count;
      end loop;
   end Read_File;
   Loaded_Evaluation : Evaluation := (1.0, 1.0, False);
   Loaded_Width : F := 0.0;
   Loaded_Hash : Unsigned_32 := 0;
   function Load (Id : Integer; Args : Args_Type; P : Configuration; X : Input; Width : F;
                  Resume : Boolean := False) return Node_Id is
      E : Evaluation;
      Cmd : constant Command := Builtin (Id);
      N : Node_Id;
   begin
      Used := 0;
      if Id < 1 or else Id > 255 then Invalid; return 0; end if;
      E := (if Resume then Loaded_Evaluation else (Target_Speed (X, P), P.Entry_Rate, True));
      if not Finite (E.Speed) or else E.Speed <= 0.0 or else not Finite (E.Entry_Rate)
        or else E.Entry_Rate <= 0.0 or else not Finite (Width) then Invalid; return 0; end if;
      Loaded_Evaluation := E; Loaded_Width := Width; Loaded_Hash := 0;
      if Cmd /= Unknown then
         case Cmd is
            when Loop_C => Set_Name ("Loop");
            when Straight_Roll => Set_Name ("Axial Roll");
            when Rolling_Circle => Set_Name ("Rolling Circle");
            when Half_Cuban => Set_Name ("Half Cuban Eight");
            when Half_Reverse_Cuban => Set_Name ("Half Reverse Cuban Eight");
            when Humpty_Bump => Set_Name ("Humpty Bump");
            when Scale_Figure_Eight => Set_Name ("Scale Figure Eight");
            when Immelmann => Set_Name ("Immelmann Turn");
            when Split_S => Set_Name ("Split-S");
            when Barrel_Roll => Set_Name ("Barrel Roll");
            when others => null;
         end case;
         return Build (Cmd, Args, Width, E);
      end if;
      declare
         Num : constant Number_Text := Decimal (Natural (Id));
         Filename : constant String := "trick" & Num.Data (Num.First .. 10) & ".txt";
      begin
         if Read_File ("APM/scripts/" & Filename) or else Read_File ("scripts/" & Filename) or else Read_File (Filename) then
            Loaded_Hash := 2166136261;
            for I in 1 .. Schedule_Length loop
               Loaded_Hash := (Loaded_Hash xor Unsigned_32 (Byte_At (I))) * 16777619;
            end loop;
            N := Parse_Schedule (Width, E); return N;
         end if;
      end;
      if Fault = OK then Check (S.INVALID); end if;
      return 0;
   end Load;
   function Rust_Integer (X : F) return Integer is
   begin
      if Is_Nan (X) then return 0;
      elsif X >= 2147483648.0 then return Integer'Last;
      elsif X <= -2147483648.0 then return Integer'First;
      else return Integer (F'Truncation (X)); end if;
   end Rust_Integer;
   function Rust_Unsigned (X : F) return Unsigned_32 is
   begin
      if Is_Nan (X) or else X <= 0.0 then return 0;
      elsif X >= 4294967296.0 then return Unsigned_32'Last;
      else return Unsigned_32 (F'Truncation (X)); end if;
   end Rust_Unsigned;
   function Mode return Unsigned_8 is
      Value : Unsigned_8 := 0;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
   begin
      if Fault /= OK then return Value; end if;
      loop
         Code := B.Vehicle.Get_Mode (Ctx, Value); exit when not Retry_Again (Code, Start);
      end loop;
      return Value;
   end Mode;
   function Mission_Command return T.Optional_Vehicle_Nav_Script_TimeOutput is
      Value : T.Optional_Vehicle_Nav_Script_TimeOutput;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
   begin
      if Fault /= OK then return Value; end if;
      loop
         Code := B.Vehicle.Nav_Script_Time (Ctx, Value); exit when not Retry_Again (Code, Start);
      end loop;
      return Value;
   end Mission_Command;
   function Mission_Index return Unsigned_16 is
      Value : Unsigned_16 := 0;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
   begin
      if Fault /= OK then return Value; end if;
      loop
         Code := B.Mission.Get_Current_Nav_Index (Ctx, Value); exit when not Retry_Again (Code, Start);
      end loop;
      return Value;
   end Mission_Index;
   function Mission_Item (Index : Unsigned_16) return T.Optional_Mavlink_Mission_Item_Int_T is
      Value : T.Optional_Mavlink_Mission_Item_Int_T;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
   begin
      if Fault /= OK then return Value; end if;
      loop
         Code := B.Mission.Get_Item (Ctx, Index, Value); exit when not Retry_Again (Code, Start);
      end loop;
      return Value;
   end Mission_Item;
   function Mission_Location (Index : Unsigned_16; Forward : Boolean) return T.Location is
      Idx : Unsigned_16 := Index;
      Item : T.Optional_Mavlink_Mission_Item_Int_T;
      Has_Location : Interfaces.C.C_bool;
      Code : Status;
      Start : Unsigned_64;
      Loc : T.Location;
   begin
      for I in 1 .. 256 loop
         Item := Mission_Item (Idx);
         if Fault /= OK then return Loc; end if;
         if not Boolean (Item.Present) then Check (S.UNAVAILABLE); return Loc; end if;
         Start := Time_Us;
         loop
            Code := B.Mission.Cmd_Has_Location (Ctx, Item.Value.Command, Has_Location);
            exit when not Retry_Again (Code, Start);
         end loop;
         if Fault /= OK then return Loc; end if;
         if Boolean (Has_Location) and then (Item.Value.X /= 0 or else Item.Value.Y /= 0) then
            Loc.Lat := Item.Value.X; Loc.Lng := Item.Value.Y; return Loc;
         end if;
         if Forward and then Item.Value.Command = 177 then
            if not Finite (Item.Value.Param1) or else Item.Value.Param1 < 0.0 or else Item.Value.Param1 > 65535.0
            then Invalid; return Loc; end if;
            Idx := Unsigned_16 (Rust_Integer (Item.Value.Param1));
         elsif Forward then
            if Idx = Unsigned_16'Last then Check (S.UNAVAILABLE); return Loc; end if;
            Idx := Idx + 1;
         else
            if Idx = 0 then Check (S.UNAVAILABLE); return Loc; end if;
            Idx := Idx - 1;
         end if;
      end loop;
      Invalid; return Loc;
   end Mission_Location;
   function Heading (Angle, Course : F) return F is
      Index : Unsigned_16;
      Previous, Following : T.Location;
      Here : T.Optional_Location;
      Yaw : F := 0.0;
      Code : Status;
      Start : Unsigned_64;
   begin
      if not Finite (Angle) or else not Finite (Course) then Invalid; return 0.0; end if;
      if Angle /= 0.0 then return Angle; end if;
      Index := Mission_Index;
      if Index > 1 then Previous := Mission_Location (Index - 1, False);
      else
         Start := Time_Us;
         loop
            exit when Fault /= OK;
            Code := B.Ahrs.Get_Location (Ctx, Here); exit when not Retry_Again (Code, Start);
         end loop;
         if Fault = OK and then not Boolean (Here.Present) then Check (S.UNAVAILABLE); end if;
         Previous := Here.Value;
      end if;
      if Index = Unsigned_16'Last then Invalid; return 0.0; end if;
      Following := Mission_Location (Index + 1, True); Start := Time_Us;
      loop
         exit when Fault /= OK;
         Code := B.Location.Get_Bearing (Ctx, Previous, Following, Yaw); exit when not Retry_Again (Code, Start);
      end loop;
      Yaw := Deg (Yaw);
      if Index > 1 and then abs Wrap_180 (Course - Yaw) > 90.0 then Yaw := Wrap_180 (Yaw + 180.0); end if;
      return Yaw;
   end Heading;
   function Rc_Action return Integer is
      Value : T.Optional_Uint8_T;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
      Option : constant T.RcChannelAuxFunc := T.RcChannelAuxFunc (Rust_Integer (Trik (3)));
   begin
      if Fault /= OK then return -1; end if;
      loop
         Code := B.Rc.Get_Aux_Cached (Ctx, Option, Value); exit when not Retry_Again (Code, Start);
      end loop;
      return (if Fault = OK and then Boolean (Value.Present) then Integer (Value.Value) else -1);
   end Rc_Action;
   procedure Enable_Control (Mode : Unsigned_8; Enabled : out Boolean) is
      Value : Interfaces.C.C_bool;
      Code : Status;
      Start : constant Unsigned_64 := Time_Us;
   begin
      Enabled := False;
      if Fault /= OK then return; end if;
      loop
         Code := B.Vehicle.Nav_Scripting_Enable (Ctx, Mode, Value); exit when not Retry_Again (Code, Start);
      end loop;
      Enabled := Fault = OK and then Boolean (Value);
   end Enable_Control;

   function Int_Bits is new Ada.Unchecked_Conversion (Integer_32, Unsigned_32);
   procedure Log_Write (Name, Labels, Format : String; Values : Floats;
                        Lat : Integer_32 := 0; Lng : Integer_32 := 0; Pose : Boolean := False) is
      Record_Data : T.LogRecord;
      Offset : Natural := 0;
      procedure Put (Value : Unsigned_32) is
      begin
         for I in 0 .. 3 loop Record_Data.Payload (Offset + I) := Unsigned_8 (Shift_Right (Value, 8 * I) and 255); end loop;
         Offset := Offset + 4;
      end Put;
   begin
      if Fault /= OK then return; end if;
      for I in Name'Range loop Record_Data.Name (I - Name'First) := Unsigned_8 (Character'Pos (Name (I))); end loop;
      for I in Labels'Range loop Record_Data.Labels (I - Labels'First) := Unsigned_8 (Character'Pos (Labels (I))); end loop;
      for I in Format'Range loop Record_Data.Format (I - Format'First) := Unsigned_8 (Character'Pos (Format (I))); end loop;
      for Value of Values loop Put (Bits (Value)); end loop;
      if Pose then Put (Int_Bits (Lat)); Put (Int_Bits (Lng)); end if;
      Record_Data.Payload_Len := Unsigned_16 (Offset);
      declare
         Code : constant Status := B.Logger.Write (Ctx, Record_Data);
      begin
         if Code /= S.UNAVAILABLE then Check (Code); end if;
      end;
   end Log_Write;
   procedure Log_Pose (Name : String; Pos : V; Rot : Q) is
      Loc : T.Optional_Location;
   begin
      if Fault /= OK then return; end if;
      Check (B.Ahrs.Get_Origin (Ctx, Loc));
      if Fault /= OK or else not Boolean (Loc.Present) then return; end if;
      Check (B.Location.Offset (Ctx, Loc.Value, Pos.X, Pos.Y));
      Log_Write (Name, "px,py,pz,q1,q2,q3,q4,r,p,y,Lat,Lon", "ffffffffffLL",
                 (Pos.X, Pos.Y, Pos.Z, Rot.Q1, Rot.Q2, Rot.Q3, Rot.Q4,
                  Deg (Euler_Roll (Rot)), Deg (Euler_Pitch (Rot)), Deg (Euler_Yaw (Rot))), Loc.Value.Lat, Loc.Value.Lng, True);
   end Log_Pose;
   procedure Log_Record (X : Input; O : Output) is
   begin
      if O.Have_Lookahead then Log_Write ("AELK", "Py,Ly,Pz,Lz", "ffff", O.Lookahead_Rates); end if;
      Log_Pose ("POSM", X.Pos, O.Measured_Attitude); Log_Pose ("POST", O.Track_Pos, O.Track_Attitude);
      Log_Write ("AETM", "T,Terr,QCt,Adt", "ffff", (O.Path_T, O.Path_Error_T, O.Q_Change_T, O.Dt));
      Log_Write ("AERT", "Cx,Cy,Cz,Px,Py,Pz,Ex,Tx,Ty,Tz,Perr,Aerr,Yff,Rofs", "ffffffffffffff",
                 (O.Correction_Rates.X, O.Correction_Rates.Y, O.Correction_Rates.Z, O.Path_Rates.X, O.Path_Rates.Y,
                  O.Path_Rates.Z, O.Roll_Error_Rate, O.Rates.X, O.Rates.Y, O.Rates.Z,
                  O.Position_Error, O.Angle_Error, 0.0, O.Rudder));
      Log_Write ("AESP", "Targ,Curr,P,I,Total,Add", "ffffff", O.Speed_Log);
   end Log_Record;
   function Run (Ctx : AP_Module.Context) return AP_Module.Status is
      Root : Node_Id := 0;
      Loaded_Id : Integer := -1;
      Loaded_Args : Args_Type := (others => 0.0);
      Active : Boolean := False;
      Language_Selected : Boolean := False;
      Shared : T.SharedStateToken;
      Shared_Revision : Unsigned_64 := 0;
      Shared_Held : Boolean := False;
      W : Snapshot := (others => 0);
      Channel : T.RC_ChannelToken;
      Last_Selection : Natural := 0;
      Last_Action : Integer := -1;
      type Owner_Kind is (No_Owner, Rc_Owner, Mission_Owner);
      Owner : Owner_Kind := No_Owner;
      Owner_Mode : Unsigned_8 := 0;
      Owner_Id : Unsigned_16 := 0;
      Last_Mission_Id, Last_Preload : Integer := -1;
      Count : Unsigned_32 := 0;
      Last_Named_Float : F := 0.0;
      Last_Error, Last_Command : Unsigned_64 := 0;
      Previous_Selection : Natural;
      Previous_Action : Integer;
      Starting, Starting_Mission, Preloading, Stepped, Delivering : Boolean;
      Attempted_Trick : Integer;
      procedure Release_Control is
         Code : Status;
         Enabled : Boolean;
      begin
         if Fault /= OK then return; end if;
         case Owner is
            when Mission_Owner => Check (B.Vehicle.Nav_Script_Time_Done (Aerobatics.Ctx, Owner_Id));
            when Rc_Owner =>
               if Mode = Owner_Mode and then Fault = OK then
                  -- Release is deliberately not retried, matching Rust.
                  declare
                     Value : Interfaces.C.C_bool;
                  begin
                     Code := B.Vehicle.Nav_Scripting_Enable (Aerobatics.Ctx, 255, Value); Check (Code);
                     Enabled := Boolean (Value);
                  end;
               end if;
            when No_Owner => null;
         end case;
         if Fault = OK then Owner := No_Owner; end if;
      end Release_Control;
      procedure Ensure_Loaded (Id : Integer; Args : Args_Type; P : Configuration; X : Input) is
      begin
         if Root = 0 or else Loaded_Id /= Id or else Loaded_Args /= Args then
            Root := 0;
            Root := Load (Id, Args, P, X, A (22));
            if Fault = OK then Loaded_Id := Id; Loaded_Args := Args; end if;
         end if;
      end Ensure_Loaded;
      function Ownership_Valid return Boolean is
         Current_Mode : constant Unsigned_8 := Mode;
         Item : T.Optional_Vehicle_Nav_Script_TimeOutput;
      begin
         if Owner = Mission_Owner and then Current_Mode = 10 then
            Item := Mission_Command;
            return Fault = OK and then Boolean (Item.Present) and then Item.Value.Out0 = Owner_Id;
         end if;
         return Owner = Rc_Owner and then Current_Mode = Owner_Mode and then Fault = OK;
      end Ownership_Valid;
      procedure Deliver (O : Output) is
         Start : Unsigned_64 := Time_Us;
         Code : Status;
      begin
         if Fault /= OK then return; end if;
         loop
            Code := B.Vehicle.Set_Target_Throttle_Rate_Rpy (Aerobatics.Ctx, O.Throttle, O.Rates.X, O.Rates.Y, O.Rates.Z);
            exit when not Retry_Again (Code, Start);
         end loop;
         if Fault /= OK then return; end if;
         Last_Command := Time_Us; Start := Time_Us;
         loop
            Code := B.Vehicle.Set_Rudder_Offset (Aerobatics.Ctx, O.Rudder, Interfaces.C.C_bool'Val (1));
            exit when not Retry_Again (Code, Start);
         end loop;
      end Deliver;
      procedure Log_Runtime (Elapsed_Us, Tick_Us : Unsigned_64; O : Output) is
         R : T.LogRecord;
         Name : constant String := "ARUN";
         Labels : constant String := "Us,Total,Perr,T,Flags";
         Format : constant String := "IIffB";
         Values : constant array (1 .. 4) of Unsigned_32 :=
           (Unsigned_32 (Unsigned_64'Min (Elapsed_Us, Unsigned_64 (Unsigned_32'Last))),
            Unsigned_32 (Unsigned_64'Min (Tick_Us, Unsigned_64 (Unsigned_32'Last))), Bits (O.Position_Error), Bits (O.Path_T));
         Code : Status;
      begin
         for I in Name'Range loop R.Name (I - 1) := Unsigned_8 (Character'Pos (Name (I))); end loop;
         for I in Labels'Range loop R.Labels (I - 1) := Unsigned_8 (Character'Pos (Labels (I))); end loop;
         for I in Format'Range loop R.Format (I - 1) := Unsigned_8 (Character'Pos (Format (I))); end loop;
         for I in Values'Range loop
            for J in 0 .. 3 loop R.Payload ((I - 1) * 4 + J) := Unsigned_8 (Shift_Right (Values (I), 8 * J) and 255); end loop;
         end loop;
         R.Payload (16) := (if O.Path_Rate_Fallback then 1 else 0) + (if O.Lookahead_Fallback then 2 else 0);
         R.Payload_Len := 17; Code := B.Logger.Write (Aerobatics.Ctx, R);
      end Log_Runtime;
      function Shared_Write return Boolean is
         Bytes : T.Array_Uint8_T_2048;
         Revision : Unsigned_64;
      begin
         if Fault /= OK then return False; end if;
         for I in W'Range loop
            for J in 0 .. 3 loop Bytes (4 * I + J) := Unsigned_8 (Shift_Right (W (I), 8 * J) and 255); end loop;
         end loop;
         Check (B.Shared_State.Write (Ctx, Shared, 0, 2048, Shared_Revision, Bytes, Revision));
         if Fault /= OK then return False; end if;
         if Revision = 0 then return False; end if;
         Shared_Revision := Revision; return True;
      end Shared_Write;
      procedure Yield_State is
         Cursor : Natural := 64;
         function Marker (Value : Integer) return Unsigned_32 is
           (if Value < 0 then Unsigned_32'Last else Unsigned_32 (Value));
      begin
         W := (others => 0); W (0) := 16#4145_524F#; W (1) := 1; W (3) := 1;
         W (4) := (if Active then 1 else 0); W (5) := Owner_Kind'Pos (Owner);
         W (6) := (if Owner = Rc_Owner then Unsigned_32 (Owner_Mode) else Unsigned_32 (Owner_Id));
         W (7) := Unsigned_32 (Last_Command and 16#FFFF_FFFF#); W (8) := Unsigned_32 (Shift_Right (Last_Command, 32));
         W (9) := Count; W (10) := Unsigned_32 (Last_Selection); W (11) := Marker (Last_Action);
         W (12) := Marker (Last_Mission_Id); W (13) := Marker (Last_Preload);
         if Active then
            W (14) := Unsigned_32 (Loaded_Id);
            for I in 1 .. 4 loop W (14 + I) := Bits (Loaded_Args (I)); end loop;
            W (19) := Bits (Loaded_Evaluation.Speed); W (20) := Bits (Loaded_Evaluation.Entry_Rate);
            W (21) := Bits (Loaded_Width); W (22) := Loaded_Hash;
            Save_Controller (W (24 .. 59)); Path_Cache (Root, W, Cursor, False);
            W (23) := Unsigned_32 ((Cursor - 64) / 4);
         end if;
         if not Shared_Write then
            if Fault = OK then Invalid; end if;
            return;
         end if;
         Shared_Held := False; Active := False; Owner := No_Owner; Root := 0; Loaded_Id := -1;
      end Yield_State;
      procedure Claim_State is
         Bytes : T.SharedStateRead;
         Cursor : Natural := 64;
         Expected_Hash : Unsigned_32;
         P : Configuration;
         X : Input;
         function Marker (Value : Unsigned_32) return Integer is
           (if Value = Unsigned_32'Last then -1 else Integer (Value));
      begin
         Check (B.Shared_State.Read (Ctx, Shared, 0, 2048, Bytes));
         if Fault /= OK then return; end if;
         Shared_Revision := Bytes.Revision; W := (others => 0);
         for I in W'Range loop
            for J in 0 .. 3 loop W (I) := W (I) or Shift_Left (Unsigned_32 (Bytes.Data (4 * I + J)), 8 * J); end loop;
         end loop;
         if W (0) = 0 then
            for Value of W loop if Value /= 0 then Invalid; return; end if; end loop;
            W (0) := 16#4145_524F#; W (1) := 1;
         end if;
         if W (0) /= 16#4145_524F# or else W (1) /= 1 then Invalid; return; end if;
         if W (2) /= 0 then return; end if;
         W (2) := Shared.Value;
         if not Shared_Write then return; end if;
         Shared_Held := True;
         if W (4) > 1 or else W (5) > 2 or else W (6) > 65535 or else W (10) > Unsigned_32 (Trick_Count)
           or else (W (11) /= Unsigned_32'Last and then W (11) > 2)
           or else (W (12) /= Unsigned_32'Last and then W (12) > 65535)
           or else (W (13) /= Unsigned_32'Last and then W (13) > 65535) then Invalid; return; end if;
         Count := W (9);
         if W (3) /= 0 then
            Last_Selection := Natural (W (10)); Last_Action := Marker (W (11));
            Last_Mission_Id := Marker (W (12)); Last_Preload := Marker (W (13));
         end if;
         Last_Command := Unsigned_64 (W (7)) or Shift_Left (Unsigned_64 (W (8)), 32);
         if W (4) = 0 then return; end if;
         if Last_Command > Time_Us then Invalid; return; end if;
         if Elapsed (Time_Us, Last_Command) > 1_000_000 then return; end if;
         if W (5) = 0 or else (W (5) = 1 and then W (6) > 255) or else W (23) > 112
           or else W (14) < 1 or else W (14) > 255 then Invalid; return; end if;
         Loaded_Evaluation := (From_Bits (W (19)), From_Bits (W (20)), False);
         for I in 1 .. 4 loop
            Loaded_Args (I) := From_Bits (W (14 + I));
            if not Finite (Loaded_Args (I)) then Invalid; return; end if;
         end loop;
         Expected_Hash := W (22); P := Read_Configuration;
         Loaded_Id := Integer (W (14)); Root := Load (Loaded_Id, Loaded_Args, P, X, From_Bits (W (21)), True);
         if Fault /= OK then return; end if;
         if Loaded_Hash /= Expected_Hash then Invalid; return; end if;
         Path_Cache (Root, W, Cursor, True);
         if Cursor /= 64 + 4 * Natural (W (23)) then Invalid; return; end if;
         Restore_Controller (W (24 .. 59));
         if Fault /= OK then return; end if;
         Owner := Owner_Kind'Val (W (5));
         if Owner = Rc_Owner then Owner_Mode := Unsigned_8 (W (6)); else Owner_Id := Unsigned_16 (W (6)); end if;
         Active := True;
      end Claim_State;
      procedure Tick is
         Tick_Start, Start, Elapsed_Us : Unsigned_64;
         Current_Mode : Unsigned_8;
         Mission_Item_Current, Check_Item : T.Optional_Vehicle_Nav_Script_TimeOutput;
         Item : T.Optional_Mavlink_Mission_Item_Int_T;
         Stale, Enabled, Finish : Boolean;
         Id, Action : Integer;
         Selection : Natural := 0;
         Args : Args_Type;
         P : Configuration;
         X : Input;
         Course, Yaw, Norm : F;
         Language : F;
         Code : Status;
         Index : Unsigned_16;
         Kind : Step_Kind;
         O : Output;
      begin
         Check (Sleep_Ms (Aerobatics.Ctx, 25));
         if Fault /= OK then return; end if;
         Language := A (35);
         if Fault /= OK then return; end if;
         if Language /= 1.0 then
            if Shared_Held then Yield_State; end if;
            Language_Selected := False; return;
         end if;
         if not Shared_Held then
            Claim_State;
            if Fault /= OK or else not Shared_Held then return; end if;
            Language := A (35);
            if Fault /= OK or else Language /= 1.0 then return; end if;
         end if;
         if not Language_Selected then
            if not Active then Last_Action := -1; end if;
            Message (6, "Aerobatics ADA selected");
            if Fault /= OK then return; end if;
            Language_Selected := True;
         end if;
         Tick_Start := Time_Us;
         if Active and then Elapsed (Tick_Start, Last_Command) > 1_000_000 then
            Message (3, "Aerobatics: control timeout"); Invalid; return;
         end if;
         if Owner /= No_Owner and then not Active then Release_Control; end if;
         Current_Mode := Mode;
         if Current_Mode = 10 then Mission_Item_Current := Mission_Command; end if;
         if Fault /= OK then return; end if;
         Stale := (Owner = Mission_Owner and then
                   (not Boolean (Mission_Item_Current.Present) or else Mission_Item_Current.Value.Out0 /= Owner_Id))
           or else (Owner = Rc_Owner and then Owner_Mode /= Current_Mode);
         if Stale then
            if Active then Count := State.Count; Active := False; end if;
            Release_Control;
         end if;
         if (Rust_Unsigned (A (34)) and 4) /= 0 then
            Release_Control; Active := False; return;
         end if;
         if Fault /= OK then return; end if;
         if Current_Mode = 10 then
            if Boolean (Mission_Item_Current.Present) then
               if Last_Mission_Id /= Integer (Mission_Item_Current.Value.Out0) then
                  Id := Integer (Mission_Item_Current.Value.Out1);
                  Args := (Mission_Item_Current.Value.Out2, Mission_Item_Current.Value.Out3,
                           F (Mission_Item_Current.Value.Out4), F (Mission_Item_Current.Value.Out5));
                  P := Read_Configuration; Validate (P); X := Observation;
                  if Fault /= OK then return; end if;
                  Last_Mission_Id := Integer (Mission_Item_Current.Value.Out0); Starting_Mission := True; Attempted_Trick := Id;
                  Ensure_Loaded (Id, Args, P, X);
                  Course := Deg (Atan2 (X.Velocity.Y, X.Velocity.X)); Yaw := Heading (A (33), Course);
                  X := Observation;
                  if Fault /= OK then return; end if;
                  Start_Controller (Root, X, P, Yaw, Count);
                  if Fault /= OK then return; end if;
                  if Mode /= 10 then Message (4, "Trick skipped: item expired"); return; end if;
                  Check_Item := Mission_Command;
                  if Fault /= OK then return; end if;
                  if not Boolean (Check_Item.Present) or else Check_Item.Value.Out0 /= Mission_Item_Current.Value.Out0 then
                     Message (4, "Trick skipped: item expired"); return;
                  end if;
                  Owner := Mission_Owner; Owner_Id := Mission_Item_Current.Value.Out0;
                  Last_Command := Time_Us; Active := True;
                  Message (6, "Starting " & Loaded_Name (1 .. Loaded_Name_Length) & "!");
               end if;
            else
               Index := Mission_Index;
               if Last_Preload /= Integer (Index) then
                  Last_Preload := Integer (Index); Preloading := True;
                  if Index /= Unsigned_16'Last then
                     Item := Mission_Item (Index + 1);
                     if Fault = OK and then Boolean (Item.Present) and then Item.Value.Command = 42702 then
                        Id := Rust_Integer (Item.Value.Param1); Attempted_Trick := Id; P := Read_Configuration;
                        Args := (Item.Value.Param3, Item.Value.Param4, F (Item.Value.X), F (Item.Value.Y));
                        Root := 0; X := Observation; Root := Load (Id, Args, P, X, A (22));
                        if Fault = OK then Loaded_Id := Id; Loaded_Args := Args; end if;
                     end if;
                  end if;
               end if;
               return;
            end if;
         else
            if Trick_Count = 0 then return; end if;
            Action := Rc_Action;
            if Fault /= OK then return; end if;
            if Last_Action = -1 and then Action /= -1 then Last_Action := Action; end if;
            if Action = 0 and then Active then
               Count := State.Count; Active := False; Last_Selection := 0; Release_Control;
               Message (1, "Trick aborted"); return;
            end if;
            if Channel.Value = 0 then
               Start := Time_Us;
               loop
                  Code := B.Rc.Find_Channel_For_Option (Aerobatics.Ctx, T.RcChannelAuxFunc (Rust_Integer (Trik (2))), Channel);
                  exit when not Retry_Again (Code, Start);
               end loop;
            end if;
            if Fault /= OK then return; end if;
            if Channel.Value /= 0 then
               Start := Time_Us;
               loop
                  Code := B.Rc_Channel.Norm_Input_Ignore_Trim (Aerobatics.Ctx, Channel, Norm);
                  exit when not Retry_Again (Code, Start);
               end loop;
               Norm := Floor (Trik (4) * Constrain (0.5 * (Norm + 1.0), 0.0, 0.999) + 1.0);
               if Norm > 0.0 and then Norm <= F (Trick_Count) then Selection := Natural (Norm); end if;
            end if;
            if Fault /= OK then return; end if;
            if not Active and then Selection > 0
              and then ((Action = 1 and then Last_Selection /= Selection) or else Action /= Last_Action)
            then
               Starting := True; Last_Selection := Selection; Last_Action := Action;
               Id := Rust_Integer (Trik (5 * Selection)); Attempted_Trick := Id;
               Args := (Trik (5 * Selection + 1), Trik (5 * Selection + 2), Trik (5 * Selection + 3), Trik (5 * Selection + 4));
               P := Read_Configuration; Validate (P);
               if Fault /= OK then return; end if;
               if Root = 0 or else Loaded_Id /= Id or else Loaded_Args /= Args then
                  X := Observation; Ensure_Loaded (Id, Args, P, X);
               end if;
               if Fault /= OK then return; end if;
               if Action = 1 then
                  declare
                     Num : constant Number_Text := Decimal (Selection);
                  begin
                     Message (6, "Trick " & Num.Data (Num.First .. 10) & " selected ("
                              & Loaded_Name (1 .. Loaded_Name_Length) & ")");
                  end;
               elsif Action = 2 then
                  X := Observation; Yaw := Wrap_180 (Deg (Atan2 (X.Velocity.Y, X.Velocity.X)));
                  if Fault /= OK then return; end if;
                  Start_Controller (Root, X, P, Yaw, Count);
                  if Fault /= OK then return; end if;
                  Enable_Control (Current_Mode, Enabled);
                  if Fault /= OK then return; end if;
                  if not Enabled then Message (1, "Tricks not available in mode"); return; end if;
                  Owner := Rc_Owner; Owner_Mode := Current_Mode; Last_Command := Time_Us;
                  declare
                     Num : constant Number_Text := Decimal (Selection);
                  begin
                     Message (6, "Trick " & Num.Data (Num.First .. 10) & " started ("
                              & Loaded_Name (1 .. Loaded_Name_Length) & ")");
                  end;
                  Last_Selection := 0; Active := True; return;
               end if;
            end if;
         end if;
         if Fault /= OK then return; end if;
         if Active then
            if Root = 0 then Invalid; return; end if;
            P := Read_Configuration; X := Observation;
            if Fault /= OK then return; end if;
            if Elapsed (Time_Us, Last_Command) > 1_000_000 then Invalid; return; end if;
            Start := Time_Us; Stepped := True; Controller_Step (Root, X, P, Kind, O);
            Count := State.Count; Elapsed_Us := Elapsed (Time_Us, Start);
            if Fault = Stopped then return;
            elsif Fault /= OK then
               Fault := OK; Message (0, "Invalid path - aborting"); Kind := Finished;
            end if;
            if Kind = Waiting then return; end if;
            Finish := Kind = Finished or else Kind = Altitude_Abort;
            if Kind = Altitude_Abort then Message (0, "Too low altitude, aborting"); end if;
            if Kind = Command_Step or else Kind = Altitude_Abort then
               Delivering := not Finish;
               if not Ownership_Valid then
                  if Fault = OK then Active := False; Owner := No_Owner; end if;
                  return;
               end if;
               Deliver (O);
               if Fault /= OK then return; end if;
               Delivering := False;
               Log_Record (X, O);
               if Fault = Stopped then return; end if;
               Fault := OK;
               if O.Path_Rate_Fallback then Message (0, "path_rate_ef_rads: invalid"); end if;
               if X.Now - Last_Named_Float > 0.5 then
                  Last_Named_Float := X.Now; Code := B.Gcs.Send_Named_Float (Aerobatics.Ctx, "PERR", O.Position_Error);
               end if;
               Log_Runtime (Elapsed_Us, Elapsed (Time_Us, Tick_Start), O);
            end if;
            if Finish then
               Release_Control;
               if Fault /= OK then return; end if;
               Message (6, "Finishing " & Loaded_Name (1 .. Loaded_Name_Length) & "!"); Active := False;
            end if;
         end if;
      end Tick;
      Error_Code, Return_Code : Status;
   begin
      Aerobatics.Ctx := Ctx; Fault := OK; Used := 0; Trick_Count := 0;
      Aerom_Names := (others => (others => <>)); Trik_Names := (others => (others => <>));
      Check (B.Shared_State.Open (Ctx, 16#4145_524F#, 2048, Shared));
      Parameters_Init;
      if Fault /= OK then
         Return_Code := Fault; Message (3, "Aerobatics parameters failed");
         Aerobatics.Ctx := System.Null_Address; return Return_Code;
      end if;
      Message (6, "Loaded plane_aerobatics Ada");
      if Trick_Count > 0 then
         declare
            Num : constant Number_Text := Decimal (Trick_Count);
         begin
            Message (6, "Enabled " & Num.Data (Num.First .. 10) & " tricks");
         end;
         Last_Action := Rc_Action;
         if Fault /= Stopped then Fault := OK; end if;
      end if;
      while Fault /= Stopped loop
         Previous_Selection := Last_Selection; Previous_Action := Last_Action;
         Starting := False; Starting_Mission := False; Preloading := False;
         Attempted_Trick := -1; Stepped := False; Delivering := False;
         Tick;
         if Fault /= OK then
            Error_Code := Fault;
            if Error_Code = S.BUSY then
               if Starting_Mission and then Owner = No_Owner then Last_Mission_Id := -1; end if;
               if Preloading then Last_Preload := -1; end if;
            end if;
            if Starting and then (Error_Code = S.BUSY or else Error_Code = S.UNAVAILABLE) and then Owner = No_Owner then
               Last_Selection := Previous_Selection; Last_Action := Previous_Action;
            end if;
            if Error_Code = S.BUSY and then (not Stepped or else Delivering) then Fault := OK;
            else
               if Active then Count := State.Count; Active := False; end if;
               Fault := OK; Release_Control;
               if Error_Code = Stopped then Fault := Stopped;
               else
                  Fault := OK;
                  if Attempted_Trick /= -1 or else Elapsed (Time_Us, Last_Error) > 1_000_000 then
                     Last_Error := Time_Us; Message (3, "Aerobatics error");
                  end if;
               end if;
            end if;
         end if;
      end loop;
      Return_Code := B.Shared_State.Close (Ctx, Shared);
      Aerobatics.Ctx := System.Null_Address; return Stopped;
   end Run;
end Aerobatics;
