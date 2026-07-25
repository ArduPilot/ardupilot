
package body MAVLink.SHA_256 is

   use type System.Bit_Order;

   K : constant State (0 .. 63) :=
     [16#428a2f98#, 16#71374491#, 16#b5c0fbcf#, 16#e9b5dba5#,
      16#3956c25b#, 16#59f111f1#, 16#923f82a4#, 16#ab1c5ed5#,
      16#d807aa98#, 16#12835b01#, 16#243185be#, 16#550c7dc3#,
      16#72be5d74#, 16#80deb1fe#, 16#9bdc06a7#, 16#c19bf174#,
      16#e49b69c1#, 16#efbe4786#, 16#0fc19dc6#, 16#240ca1cc#,
      16#2de92c6f#, 16#4a7484aa#, 16#5cb0a9dc#, 16#76f988da#,
      16#983e5152#, 16#a831c66d#, 16#b00327c8#, 16#bf597fc7#,
      16#c6e00bf3#, 16#d5a79147#, 16#06ca6351#, 16#14292967#,
      16#27b70a85#, 16#2e1b2138#, 16#4d2c6dfc#, 16#53380d13#,
      16#650a7354#, 16#766a0abb#, 16#81c2c92e#, 16#92722c85#,
      16#a2bfe8a1#, 16#a81a664b#, 16#c24b8b70#, 16#c76c51a3#,
      16#d192e819#, 16#d6990624#, 16#f40e3585#, 16#106aa070#,
      16#19a4c116#, 16#1e376c08#, 16#2748774c#, 16#34b0bcb5#,
      16#391c0cb3#, 16#4ed8aa4a#, 16#5b9cca4f#, 16#682e6ff3#,
      16#748f82ee#, 16#78a5636f#, 16#84c87814#, 16#8cc70208#,
      16#90befffa#, 16#a4506ceb#, 16#bef9a3f7#, 16#c67178f2#];

   function Sigma0
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 with Inline;

   function Sigma1
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 with Inline;

   function S0
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 with Inline;

   function S1
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 with Inline;

   function Swap
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32;

   ------------
   -- Sigma0 --
   ------------

   function Sigma0
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 is
   begin
      return Rotate_Right (X, 2)
         xor Rotate_Right (X, 13)
         xor Rotate_Right (X, 22);
   end Sigma0;

   ------------
   -- Sigma1 --
   ------------

   function Sigma1
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 is
   begin
      return Rotate_Right (X, 6)
         xor Rotate_Right (X, 11)
         xor Rotate_Right (X, 25);
   end Sigma1;

   --------
   -- S0 --
   --------

   function S0
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 is
   begin
      return Rotate_Right (X, 7)
         xor Rotate_Right (X, 18)
         xor Shift_Right  (X, 3);
   end S0;

   --------
   -- S1 --
   --------

   function S1
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 is
   begin
      return Rotate_Right (X, 17)
         xor Rotate_Right (X, 19)
         xor Shift_Right  (X, 10);
   end S1;

   ----------
   -- Swap --
   ----------

   function Swap
     (X : Interfaces.Unsigned_32)
      return Interfaces.Unsigned_32 is
   begin
      return (Shift_Left (X, 24) and 16#ff000000#) or
        (Shift_Left (X, 8) and 16#00ff0000#) or
        (Shift_Right (X, 8) and 16#0000ff00#) or
        (Shift_Right (X, 24) and 16#000000ff#);
   end Swap;

   ---------------
   -- Transform --
   ---------------

   procedure Transform (Self : in out Context)
   is

      function Ch
        (X, Y, Z : Interfaces.Unsigned_32)
         return Interfaces.Unsigned_32 with Inline;
      function Maj
        (X, Y, Z : Interfaces.Unsigned_32)
         return Interfaces.Unsigned_32 with Inline;

      --------
      -- Ch --
      --------

      function Ch
        (X, Y, Z : Interfaces.Unsigned_32)
         return Interfaces.Unsigned_32 is
      begin
         return (X and Y) xor ((not X) and Z);
      end Ch;

      ---------
      -- Maj --
      ---------

      function Maj
        (X, Y, Z : Interfaces.Unsigned_32)
         return Interfaces.Unsigned_32 is
      begin
         return (X and Y) xor (X and Z) xor (Y and Z);
      end Maj;

      X : State (0 .. 15) with Import, Address => Self.Buffer'Address;
      W : State (0 .. 63);

      A, B, C, D, E, F, G, H, T1, T2 : Interfaces.Unsigned_32;

      --  Start of processing for Transform

   begin
      if System.Default_Bit_Order /= System.High_Order_First then
         for J in X'Range loop
            X (J) := Swap (X (J));
         end loop;
      end if;

      --  1. Prepare message schedule

      W (0 .. 15) := X;

      for T in 16 .. 63 loop
         W (T) := S1 (W (T - 2)) + W (T - 7) + S0 (W (T - 15)) + W (T - 16);
      end loop;

      --  2. Initialize working variables

      A := Self.H_State (0);
      B := Self.H_State (1);
      C := Self.H_State (2);
      D := Self.H_State (3);
      E := Self.H_State (4);
      F := Self.H_State (5);
      G := Self.H_State (6);
      H := Self.H_State (7);

      --  3. Perform transformation rounds

      for T in 0 .. 63 loop
         T1 := H + Sigma1 (E) + Ch (E, F, G) + K (T) + W (T);
         T2 := Sigma0 (A) + Maj (A, B, C);
         H := G;
         G := F;
         F := E;
         E := D + T1;
         D := C;
         C := B;
         B := A;
         A := T1 + T2;
      end loop;

      --  4. Update hash state

      Self.H_State (0) := A + Self.H_State (0);
      Self.H_State (1) := B + Self.H_State (1);
      Self.H_State (2) := C + Self.H_State (2);
      Self.H_State (3) := D + Self.H_State (3);
      Self.H_State (4) := E + Self.H_State (4);
      Self.H_State (5) := F + Self.H_State (5);
      Self.H_State (6) := G + Self.H_State (6);
      Self.H_State (7) := H + Self.H_State (7);
   end Transform;

      ------------
      -- Update --
      ------------

   procedure Update
     (Self  : in out Context;
      Input : Data)
   is
      First, Last : Natural;

   begin
      if Input'Length = 0 then
         return;
      end if;

      Self.Length := Self.Length + Input'Length;

      First := Input'First;
      loop
         declare
            Length : constant Natural := Natural'Min
              (Self.Buffer'Last - Self.Last, Input'Last - First + 1);

         begin
            pragma Assert (Length > 0);

            Self.Buffer (Self.Last + 1 .. Self.Last + Length) :=
              Input (First .. First + Length - 1);

            Self.Last := Self.Last + Length;
            Last      := First + Length - 1;
         end;

         if Self.Last = Self.Buffer'Last then
            Transform (Self);
            Self.Last := 0;
         end if;

         exit when Last = Input'Last;
         First := Last + 1;
      end loop;
   end Update;

   ------------
   -- Digest --
   ------------

   procedure Digest
     (Self : in out Context;
      Res  : out Digest_Type)
   is
      Message_Length : Interfaces.Unsigned_64 := Self.Length;
      Zeroes         : constant Natural :=
        (Self.Buffer'Last - 9 - Self.Last) mod Self.Buffer'Length;

   begin
      declare
         Pad         : Data (1 .. 9 + Zeroes) := [1 => 128, others => 0];
         Index       : Natural;
         First_Index : Natural;

      begin
         First_Index := Pad'Last;
         Index       := First_Index;

         while Message_Length > 0 loop
            if Index = First_Index then
               Pad (Index) := Interfaces.Unsigned_8
                 (Shift_Left (Message_Length and 16#1f#, 3));
               Message_Length := Shift_Right (Message_Length, 5);

            else
               Pad (Index) := Interfaces.Unsigned_8
                 (Message_Length and 16#ff#);
               Message_Length := Shift_Right (Message_Length, 8);
            end if;

            Index := Index - 1;
         end loop;

         Update (Self, Pad);
      end;

      pragma Assert (Self.Last = 0);

      declare
         Hash_Words : constant Natural := Self.H_State'Size / 32;
         Result     : State (1 .. Hash_Words) := Self.H_State
           (Self.H_State'Last - Hash_Words + 1 .. Self.H_State'Last);
         R_SEA      : Data (1 .. Result'Size / 8) with Import,
           Address => Result'Address;
      begin
         if System.Default_Bit_Order /= System.Low_Order_First then
            for J in Result'Range loop
               Result (J) := Swap (Result (J));
            end loop;
         end if;

         --  Return truncated hash
         Res := R_SEA (R_SEA'First .. R_SEA'First + Digest_Type'Length - 1);
      end;
   end Digest;

end MAVLink.SHA_256;
