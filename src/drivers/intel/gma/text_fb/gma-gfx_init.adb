-- SPDX-License-Identifier: GPL-2.0-only

with HW.GFX;
with HW.GFX.GMA;
with HW.GFX.GMA.Display_Probing;

use HW.GFX;
use HW.GFX.GMA;
use HW.GFX.GMA.Display_Probing;

with GMA.Mainboard;

package body GMA.GFX_Init
is
   configs : Pipe_Configs;

   procedure gfxinit (lightup_ok : out Interfaces.C.int)
   is
      ports : Port_List;

      success : boolean;

      -- from pc80/vga driver
      procedure vga_io_init;
      pragma Import (C, vga_io_init, "vga_io_init");
      procedure vga_textmode_init;
      pragma Import (C, vga_textmode_init, "vga_textmode_init");
   begin
      lightup_ok := 0;

      HW.GFX.GMA.Initialize (Success => success);

      if success then
         ports := Mainboard.ports;
         HW.GFX.GMA.Display_Probing.Scan_Ports
           (Configs  => configs,
            Ports    => ports,
            Max_Pipe => Primary);

         -- Find the first active pipe. On i945, LVDS_Needs_Pipe_B
         -- causes LVDS to be swapped from Primary to Secondary,
         -- so we cannot assume Primary always has a display.
         declare
            First_Pipe : Pipe_Index := Primary;
            Has_Display : Boolean := False;
         begin
            for i in Pipe_Index loop
               if configs (i).Port /= Disabled then
                  First_Pipe := i;
                  Has_Display := True;
                  exit;
               end if;
            end loop;

            if Has_Display then
               HW.GFX.GMA.Power_Up_VGA;
               vga_io_init;
               vga_textmode_init;

               -- override probed framebuffer config
               configs (First_Pipe).Framebuffer.Width    := 640;
               configs (First_Pipe).Framebuffer.Height   := 400;
               configs (First_Pipe).Framebuffer.Offset   :=
                  VGA_PLANE_FRAMEBUFFER_OFFSET;

               pragma Debug (HW.GFX.GMA.Dump_Configs (configs));
               HW.GFX.GMA.Update_Outputs (configs);

               lightup_ok := 1;
            end if;
         end;
      end if;
   end gfxinit;

   procedure gfxstop
   is
   begin
      for i in Pipe_Index loop
         if configs (i).Port /= Disabled then
            for j in Pipe_Index loop
               configs (j).Port := Disabled;
            end loop;
            HW.GFX.GMA.Update_Outputs (configs);
            exit;
         end if;
      end loop;
   end gfxstop;

end GMA.GFX_Init;
