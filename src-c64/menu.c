#include <conio.h>
#include <stdint.h>

#include <stdio.h>

#include "../commands.h"
#include "icif.h"


uint8_t selected;
static uint8_t textc = COLOR_LIGHTGREEN;

static const char *program = "Initial Cardridge v0.10";

static void menu_basic(void);
static void menu_files(void);
static void menu_fb64(void);
static void menu_setting(void);

struct elem {
  char name[16];
  void (*func)(void);
};

struct elem menu_elems[] = {
    {"QUIT TO BASIC", menu_basic}, {"SD CARD FILES", menu_files}, {"FB64", menu_fb64}
    //, {"SETTINGS", menu_setting}
};

#define no_of_elements ((sizeof(menu_elems))/(sizeof(struct elem)))

#define DIRH no_of_elements
#define DIRW 20

#define MENUX 27
#define MENUY 1
#define MENUW 13
#define MENUH 23

#define DIR1X 0
#define DIR1Y 1

#define SCREENH 25
#define BOTTOM (SCREENH - 1)

#define SCREENW 40

static void menu_basic(void) {
    DirElement elem = {
        0, "basic", 0xa0
    };
    ic_select_boot(&elem);
    //  __asm__("jmp $fce2"); // Soft reset
}

static void menu_files(void) {
    DirElement elem = {
        0, "browser", FILE_PRG
    };
    ic_select_boot(&elem);
}

static void menu_fb64(void) {
    DirElement elem = {
        0, "fb64", FILE_PRG
    };
    ic_select_boot(&elem);
}

static void menu_setting(void) {

}

static void drawFrame(int xpos, int ypos, int xsize, int ysize) {
  // top
  gotoxy(xpos, ypos);
  cputc(CH_ULCORNER);

  chline(xsize - 2);
  cputc(CH_URCORNER);

  // left
  cvlinexy(xpos, ypos + 1, ysize - 2);

  // bottom
  cputc(CH_LLCORNER);
  chline(xsize - 2);
  cputc(CH_LRCORNER);

  // right
  cvlinexy(xpos + xsize - 1, ypos + 1, ysize - 2);
}

static void printElement(uint16_t pos, uint8_t reversed) {
  struct elem *current = menu_elems + pos;
  uint8_t page = 0;
  uint16_t idx = 0;
  uint16_t element = 0;
  uint16_t yoff = 0;

  yoff = pos % DIRH;
  gotoxy(DIR1X + 1, DIR1Y + 1 + yoff);
  if (reversed) {
    revers(1);
  }
  // textcolor(COLOR_GREEN);
  cprintf("%-20s", current->name);
  if (reversed) {
    revers(0);
  }
}

static void printDir(void) {
  uint8_t idx;
  for (idx = 0; idx < no_of_elements; idx++) {
    gotoxy(DIR1X + 1, DIR1Y + 1 + idx);
    if (idx == selected) {
      revers(1);
    }
    cprintf("%-18s", menu_elems[idx].name);
    if (idx == selected) {
      revers(0);
    }
  }

  /*
  // clear empty lines
  for (; idx < DIRH; idx++) {
    gotoxy(DIR1X + 1, DIR1Y + 1 + idx);
    cputs("                  ");
  }
  */
}

/*
static void updateMenu(void) {
  gotoxy(MENUX + 1, MENUY + 2);
  cputs(" CR SELECT");
  gotoxy(MENUX + 1, MENUY + 4);
  cputs(" Q  QUIT");

  drawFrame(MENUX, MENUY, MENUW, MENUH + 1);
}
*/

static void mainLoop(void) {
  uint8_t c, last_selected, exitflag = 0;
  struct elem *current;

  uint8_t menuy = MENUY;
  clrscr();
  textcolor(textc);
  revers(0);
  //updateMenu();

  textcolor(COLOR_WHITE);
  drawFrame(DIR1X, DIR1Y, DIRW + 2, DIRH + 2);

  revers(1);
  gotoxy(1, 0);
  cputs(program);
  revers(0);

  textcolor(textc);
  printDir();


  do {
    last_selected = selected;
    c = kbhit() ? cgetc() : 0;

    switch (c) {
    // --- start / enter directory
    case CH_ENTER:
      current = menu_elems + selected;
      current->func();
      break;

    // --- root directory
    case CH_HOME:
      selected = 0;
      break;

    case CH_CURS_DOWN:
      if ((selected + 1) < no_of_elements) {
        selected++;
      }
      break;

    case CH_CURS_UP:
      if (selected) {
        selected--;
      }
      break;

    case CH_CURS_RIGHT:
      break;

    case CH_CURS_LEFT:
      break;

    case 'q':
      exitflag = 1;
      break;
    }

    if (selected != last_selected) {
        printElement(last_selected, 0);
        printElement(selected, 1);
    }

  } while (exitflag == 0);
}

int main(void) {
  clrscr();
  ((uint8_t *)(204))[0] = 255;
  mainLoop();
  __asm__("jmp $fce2"); // Soft reset
  return 0;
}