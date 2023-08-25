#include <stdio.h>

#include "Si5338-RevB_RADIANT_INTERNAL-Registers_2.h"

int main() {

  int nregs = 0; 
	for (int i=0;i<NUM_REGS_MAX;i++) 
  {
		if (Reg_Store[i].Reg_Mask != 0x00)
    {
      nregs++; 
    }
  }

  int ireg = 0;
  printf("#ifdef _BM_CLOCKS\n#error only one clock allowed!\n#endif\n\n#define _BM_CLOCKS\n#define NCLOCK_REGS %d\n\nconst uint8_t CLOCK_REGS[NCLOCK_REGS][3] =\n{\n", nregs); 
	for (int i=0;i<NUM_REGS_MAX;i++) 
  {
		if (Reg_Store[i].Reg_Mask != 0x00)
    {
			printf("  {0x%02x,0x%02x,0x%02x}%s\n", Reg_Store[i].Reg_Addr, Reg_Store[i].Reg_Val, Reg_Store[i].Reg_Mask, ireg < nregs-1 ? ",": "" );		
      ireg++;
    }
	}
  printf("};\n"); 
}
