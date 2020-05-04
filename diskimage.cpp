//#include <stdio.h>
#include <SdFat.h>
//#include <stdlib.h>
#include <string.h>
#include "diskimage.h"

extern SdFat SD;

typedef struct errormessage {
  signed int number;
  char *string;
} ErrorMessage;


ErrorMessage error_msg[] = {
  /* non-errors */
  { 0, "ok" },
  { 1, "files scratched" },
  { 2, "partition selected" },
  /* errors */
  { 20, "read error (block header not found)" },
  { 21, "read error (drive not ready)" },
  { 22, "read error (data block not found)" },
  { 23, "read error (crc error in data block)" },
  { 24, "read error (byte sector header)" },
  { 25, "write error (write-verify error)" },
  { 26, "write protect on" },
  { 27, "read error (crc error in header)" },
  { 30, "syntax error (general syntax)" },
  { 31, "syntax error (invalid command)" },
  { 32, "syntax error (long line)" },
  { 33, "syntax error (invalid file name)" },
  { 34, "syntax error (no file given)" },
  { 39, "syntax error (invalid command)" },
  { 50, "record not present" },
  { 51, "overflow in record" },
  { 52, "file too large" },
  { 60, "write file open" },
  { 61, "file not open" },
  { 62, "file not found" },
  { 63, "file exists" },
  { 64, "file type mismatch" },
  { 65, "no block" },
  { 66, "illegal track or sector" },
  { 67, "illegal system t or s" },
  { 70, "no channel" },
  { 71, "directory error" },
  { 72, "disk full" },
  { 73, "dos mismatch" },
  { 74, "drive not ready" },
  { 75, "format error" },
  { 76, "controller error" },
  { 77, "selected partition illegal" },
  { -1, NULL }
};


/* dos errors as used by the DOS internally (and as saves in the error info) */
typedef struct doserror {
  signed int number;  /* dos error number */
  signed int errnum;  /* reported error number */
  char *string;       /* description */
} DosError;

DosError dos_error[] = {
  /* non-errors */
  { 0x01,  0, "ok" },
  /* errors */
  { 0x02, 20, "Header descriptor byte not found (Seek)" },
/*  { 0x02, 20, "Header block not found (Seek)" }, */
  { 0x03, 21, "No SYNC sequence found (Seek)" },
  { 0x04, 22, "Data descriptor byte not found (Read)" },
  { 0x05, 23, "Checksum error in data block (Read)" },
  { 0x06, 24, "Write verify (Write)" },
  { 0x07, 25, "Write verify error (Write)" },
  { 0x08, 26, "Write protect on (Write)" },
  { 0x09, 27, "Checksum error in header block (Seek)" },
  { 0x0A, 28, "Write error (Write)" },
  { 0x0B, 29, "Disk sector ID mismatch (Seek)" },
  { 0x0F, 74, "Drive Not Ready (Read)" },
  { -1, -1, NULL }
};


/* convert to rawname */
int di_rawname_from_name(unsigned char *rawname, char *name) {
  int i;

  memset(rawname, 0xa0, 16);
  for (i = 0; i < 16 && name[i]; ++i) {
    rawname[i] = name[i];
  }
  return i;
}


/* convert from rawname */
int di_name_from_rawname(char *name, unsigned char *rawname) {
  int i;

  for (i = 0; i < 16 && rawname[i] != 0xa0; ++i) {
    name[i] = rawname[i];
  }
  name[i] = 0;
  return i;
}


/* return status string */
int di_status(DiskImage *di, char *status) {
  ErrorMessage *err = error_msg;

  /* special case for power up */
  if (di->status == 254) {
    switch (di->type) {
    case D64:
      strcpy(status, "73,cbm dos v2.6 1541,00,00");
      break;
    case D71:
      strcpy(status, "73,cbm dos v3.0 1571,00,00");
      break;
    case D81:
      strcpy(status, "73,copyright cbm dos v10 1581,00,00");
      break;
    }
    return 73;
  }

  while (err->number >= 0) {
    if (di->status == err->number) {
      sprintf(status, "%02d,%s,%02d,%02d", di->status, err->string, di->statusts.track, di->statusts.sector);
      return di->status;
    }
    ++err;
  }
  sprintf(status, "%02d,unknown error,%02d,%02d", di->status, di->statusts.track, di->statusts.sector);
  return di->status;
}


int set_status(DiskImage *di, int status, int track, int sector) {
  di->status = status;
  di->statusts.track = track;
  di->statusts.sector = sector;
  return status;
}


/* return write interleave */
int interleave(ImageType type) {
  switch (type) {
  case D64:
    return 10;
    break;
  case D71:
    return 6;
    break;
  default:
    return 1;
    break;
  }
}


/* return number of tracks for image type */
int di_tracks(ImageType type) {
  switch (type) {
  case D64:
    return 35;
    break;
  case D71:
    return 70;
    break;
  case D81:
    return 80;
    break;
  }
  return 0;
}


/* return disk geometry for track */
int di_sectors_per_track(ImageType type, int track) {
  switch (type) {
  case D71:
    if (track > 35) {
      track -= 35;
    }
    /* fall through */
  case D64:
    if (track < 18) {
      return 21;
    } else if (track < 25) {
      return 19;
    } else if (track < 31) {
      return 18;
    } else {
      return 17;
    }
    break;
  case D81:
    return 40;
    break;
  }
  return 0;
}


/* check if given track/sector is within valid range */
int di_ts_is_valid(ImageType type, TrackSector ts) {
  if ((ts.track < 1) || (ts.track > di_tracks(type))) {
    return 0; /* track out of range */
  }
  if (ts.sector > (di_sectors_per_track(type, ts.track) - 1)) {
    return 0; /* sector out of range */
  }
  return 1;
}


/* convert track, sector to blocknum */
int get_block_num(ImageType type, TrackSector ts) {
  int block;

  /* assertion, should never happen (indicates bad error handling elsewhere) */
  if (! di_ts_is_valid(type, ts)) {
    Serial.print("ERR internal error, track/sector out of range");
    return -1;
  }

  switch (type) {
  case D64:
    if (ts.track < 18) {
      block = (ts.track - 1) * 21;
    } else if (ts.track < 25) {
      block = (ts.track - 18) * 19 + 17 * 21;
    } else if (ts.track < 31) {
      block = (ts.track - 25) * 18 + 17 * 21 + 7 * 19;
    } else {
      block = (ts.track - 31) * 17 + 17 * 21 + 7 * 19 + 6 * 18;
    }
    return block + ts.sector;
    break;
  case D71:
    if (ts.track > 35) {
      block = 683;
      ts.track -= 35;
    } else {
      block = 0;
    }
    if (ts.track < 18) {
      block += (ts.track - 1) * 21;
    } else if (ts.track < 25) {
      block += (ts.track - 18) * 19 + 17 * 21;
    } else if (ts.track < 31) {
      block += (ts.track - 25) * 18 + 17 * 21 + 7 * 19;
    } else {
      block += (ts.track - 31) * 17 + 17 * 21 + 7 * 19 + 6 * 18;
    }
    return block + ts.sector;
    break;
  case D81:
    return (ts.track - 1) * 40 + ts.sector;
    break;
  }
  return 0;
}



/* get a pointer to block data */
unsigned char *di_get_ts_addr(DiskImage *di, TrackSector ts) {
  int block = get_block_num(di->type, ts);
  if (block != di->currentblock) {
    di->file.seek(block * 256L);
    int len = di->file.read(di->blkbuffer, 256);
    if (len > 0) {
    // FIXME handle read error
    di->currentblock = block;
    }
  }
  return di->blkbuffer;
}


/* get error info for a sector */
int get_ts_doserr(DiskImage *di, TrackSector ts) {
  //if (di->errinfo == NULL) {
    return 1; /* return OK if image has no error info */
  //}

  //return di->errinfo[get_block_num(di->type, ts)];
}


/* get status info for a sector */
int di_get_ts_err(DiskImage *di, TrackSector ts) {
  int errnum;
  DosError *err = dos_error;

  errnum = get_ts_doserr(di,ts);
  while (err->number >= 0) {
    if (errnum == err->number) {
      return err->errnum;
    }
    ++err;
  }
  return -1; /* unknown error */
}


/* return a pointer to the next block in the chain */
TrackSector next_ts_in_chain(DiskImage *di, TrackSector ts) {
  unsigned char *p;
  TrackSector newts;

  p = di_get_ts_addr(di, ts);
  newts.track = p[0];
  newts.sector = p[1];

  return newts;
}

/* return t/s of first directory sector */
TrackSector di_get_dir_ts(DiskImage *di) {
  TrackSector newts;
  unsigned char *p;

  p = di_get_ts_addr(di, di->dir);
  if ((di->type == D64) || (di->type == D71)) {
    newts.track = 18; /* 1541/71 ignores bam t/s link */
    newts.sector = 1;
  } else {
    newts.track = p[0];
    newts.sector = p[1];
  }

  return newts;
}


/* return t/s of first bam sector */
TrackSector di_get_bam_ts(DiskImage *di) {
  return di->bam;
}


/* return a pointer to the disk title */
unsigned char *di_title(DiskImage *di) {
  switch (di->type) {
  default:
  case D64:
  case D71:
    return di_get_ts_addr(di, di->dir) + 144;
    break;
  case D81:
    return di_get_ts_addr(di, di->dir) + 4;
    break;
  }
}


/* return number of free blocks in track */
int di_track_blocks_free(DiskImage *di, int track) {
  unsigned char *bam;

  switch (di->type) {
  default:
  case D64:
    bam = di_get_ts_addr(di, di->bam);
    break;
  case D71:
    bam = di_get_ts_addr(di, di->bam);
    if (track >= 36) {
      return bam[track + 185];
    }
    break;
  case D81:
    if (track <= 40) {
      bam = di_get_ts_addr(di, di->bam);
    } else {
      bam = di_get_ts_addr(di, di->bam2);
      track -= 40;
    }
    return bam[track * 6 + 10];
    break;
  }
  return bam[track * 4];
}


/* count number of free blocks */
int blocks_free(DiskImage *di) {
  int track;
  int blocks = 0;

  for (track = 1; track <= di_tracks(di->type); ++track) {
    if (track != di->dir.track) {
      blocks += di_track_blocks_free(di, track);
    }
  }
  return blocks;
}


/* check if track, sector is free in BAM */
int di_is_ts_free(DiskImage *di, TrackSector ts) {
  unsigned char mask;
  unsigned char *bam;

  switch (di->type) {
  case D64:
    bam = di_get_ts_addr(di, di->bam);
    if (bam[ts.track * 4]) {
      mask = 1<<(ts.sector & 7);
      return bam[ts.track * 4 + ts.sector / 8 + 1] & mask ? 1 : 0;
    } else {
      return 0;
    }
    break;
  case D71:
    mask = 1<<(ts.sector & 7);
    if (ts.track < 36) {
      bam = di_get_ts_addr(di, di->bam);
      return bam[ts.track * 4 + ts.sector / 8 + 1] & mask ? 1 : 0;
    } else {
      bam = di_get_ts_addr(di, di->bam2);
      return bam[(ts.track - 35) * 3 + ts.sector / 8 - 3] & mask ? 1 : 0;
    }
    break;
  case D81:
    mask = 1<<(ts.sector & 7);
    if (ts.track < 41) {
      bam = di_get_ts_addr(di, di->bam);
    } else {
      bam = di_get_ts_addr(di, di->bam2);
      ts.track -= 40;
    }
    return bam[ts.track * 6 + ts.sector / 8 + 11] & mask ? 1 : 0;
    break;
  }
  return 0;
}




bool di_load_image(DiskImage *di, const char *name) {
  File file;
  long filesize;
 
  /* open image */
  file = SD.open(name);
  if (!file) {
    Serial.println("ERR FILE NOT FOUND");
    return false;
  }

  /* get file size*/
  filesize = file.fileSize();

  di->file = file;
  di->size = filesize;

  //di->errinfo = NULL;

  /* check image type */
  switch (filesize) {
  case D64ERRSIZE: /* D64 with error info */
    //di->errinfo = &(di->image[D64SIZE]);
  case D64SIZE: /* standard D64 */
    di->type = D64;
    di->bam.track = 18;
    di->bam.sector = 0;
    di->dir = di->bam;
    break;

  case D71ERRSIZE: /* D71 with error info */
    //di->errinfo = &(di->image[D71SIZE]);
  case D71SIZE:
    di->type = D71;
    di->bam.track = 18;
    di->bam.sector = 0;
    di->bam2.track = 53;
    di->bam2.sector = 0;
    di->dir = di->bam;
    break;

  case D81ERRSIZE: /* D81 with error info */
    //di->errinfo = &(di->image[D81SIZE]);
  case D81SIZE:
    di->type = D81;
    di->bam.track = 40;
    di->bam.sector = 1;
    di->bam2.track = 40;
    di->bam2.sector = 2;
    di->dir.track = 40;
    di->dir.sector = 0;
    break;

  default:
    di->file.close();
    return NULL;
  }

  di->openfiles = 0;
  di->blocksfree = blocks_free(di);
  //di->modified = 0;
  di->interleave = interleave(di->type);
  set_status(di, 254, 0, 0);
  return di;
}



void di_free_image(DiskImage *di) {
  di->file.close();
}


int match_pattern(unsigned char *rawpattern, unsigned char *rawname) {
  int i;

  for (i = 0; i < 16; ++i) {
    if (rawpattern[i] == '*') {
      return 1;
    }
    if (rawname[i] == 0xa0) {
      if (rawpattern[i] == 0xa0) {
        return 1;
      } else {
        return 0;
      }
    } else {
      if (rawpattern[i] == '?' || rawpattern[i] == rawname[i]) {
      } else {
        return 0;
      }
    }
  }
  return 1;
}


RawDirEntry *find_file_entry(DiskImage *di, unsigned char *rawpattern, FileType type) {
  unsigned char *buffer;
  TrackSector ts;
  RawDirEntry *rde;
  int offset;

  ts = di_get_dir_ts(di);

  while (ts.track) {
    buffer = di_get_ts_addr(di, ts);
    for (offset = 0; offset < 256; offset += 32) {
      rde = (RawDirEntry *)(buffer + offset);
      if ((rde->type & 0x07) == (type)) {
        if (match_pattern(rawpattern, rde->rawname)) {
          return rde;
        }
      }
    }
    /* todo: add sanity checking */
    ts = next_ts_in_chain(di, ts);
  }
  return NULL;
}


/* open a file */
bool di_open(DiskImage *di, ImageFile *imgfile, unsigned char *rawname, FileType type, const char *mode) {
  RawDirEntry *rde = NULL;
  unsigned char *p;

  set_status(di, 255, 0, 0);

  if (strcmp("rb", mode) == 0) {

    memset(imgfile->visited, 0, sizeof(imgfile->visited));

    if (strcmp("$", (char *) rawname) == 0) {

      imgfile->ts = di->dir;

      p = di_get_ts_addr(di, di->dir);
      imgfile->buffer = p + 2;

      imgfile->nextts = di_get_dir_ts(di);

      imgfile->buflen = 254;

      if (! di_ts_is_valid(di->type, imgfile->nextts)) {
        set_status(di, 66, imgfile->nextts.track, imgfile->nextts.sector);
        return false;
      }

    } else {
      if ((rde = find_file_entry(di, rawname, type)) == NULL) {
        set_status(di, 62, 0, 0);
        return false;
      }
      imgfile->ts = rde->startts;

      if (! di_ts_is_valid(di->type, imgfile->ts)) {
        set_status(di, 66, imgfile->ts.track, imgfile->ts.sector);
        return false;
      }

      p = di_get_ts_addr(di, rde->startts);
      imgfile->buffer = p + 2;
      imgfile->nextts.track = p[0];
      imgfile->nextts.sector = p[1];

      if (imgfile->nextts.track == 0) {
        if (imgfile->nextts.sector != 0) {
          imgfile->buflen = imgfile->nextts.sector - 1;
        } else {
          imgfile->buflen = 254;
        }
      } else {
        if (! di_ts_is_valid(di->type,imgfile->nextts)) {
          set_status(di, 66, imgfile->nextts.track, imgfile->nextts.sector);
          return NULL;
        }
        imgfile->buflen = 254;
      }
    }

  } else if (strcmp("wb", mode) == 0) {
  } else {
    return false;
  }

  imgfile->diskimage = di;
  imgfile->rawdirentry = rde;
  imgfile->position = 0;
  imgfile->bufptr = 0;

  ++(di->openfiles);
  set_status(di, 0, 0, 0);
  return true;
}


int di_read(ImageFile *imgfile, unsigned char *buffer, int len) {
  unsigned char *p;
  int bytesleft;
  int counter = 0;
  int err;

  while (len) {
    bytesleft = imgfile->buflen - imgfile->bufptr;

    err = di_get_ts_err(imgfile->diskimage, imgfile->ts);
    if (err) {
      set_status(imgfile->diskimage, err, imgfile->ts.track, imgfile->ts.sector);
      return counter;
    }

    if (bytesleft == 0) {
      if (imgfile->nextts.track == 0) {
        return counter;
      }
      if (((imgfile->diskimage->type == D64) || (imgfile->diskimage->type == D71)) && imgfile->ts.track == 18 && imgfile->ts.sector == 0) {
        imgfile->ts.track = 18;
        imgfile->ts.sector = 1;
      } else {
        imgfile->ts = next_ts_in_chain(imgfile->diskimage, imgfile->ts);
      }
      if (imgfile->ts.track == 0) {
        return counter;
      }

      /* check for cyclic files */
      if (imgfile->visited[imgfile->ts.track][imgfile->ts.sector/8] & (1<<(imgfile->ts.sector&7))) {
        /* return 52, file too long error */
        set_status(imgfile->diskimage, 52, imgfile->ts.track, imgfile->ts.sector);
      } else {
        imgfile->visited[imgfile->ts.track][imgfile->ts.sector/8] |= (1<<(imgfile->ts.sector&7));
      }

      err = di_get_ts_err(imgfile->diskimage, imgfile->ts);
      if(err) {
        set_status(imgfile->diskimage, err, imgfile->ts.track, imgfile->ts.sector);
        return counter;
      }

      p = di_get_ts_addr(imgfile->diskimage, imgfile->ts);
      imgfile->buffer = p + 2;
      imgfile->nextts.track = p[0];
      imgfile->nextts.sector = p[1];

      if (imgfile->nextts.track == 0) {

        if (imgfile->nextts.sector == 0) {
          /* fixme, something is wrong if this happens, should be a proper error */
          imgfile->buflen = 0;
          set_status(imgfile->diskimage, -1, imgfile->ts.track, imgfile->ts.sector);
        } else {
          imgfile->buflen = imgfile->nextts.sector - 1;
        }

      } else {

        if (! di_ts_is_valid(imgfile->diskimage->type, imgfile->nextts)) {
          set_status(imgfile->diskimage, 66, imgfile->nextts.track, imgfile->nextts.sector);
          return counter;
        }

        imgfile->buflen = 254;
      }
      imgfile->bufptr = 0;
    } else {
      if (len >= bytesleft) {
        while (bytesleft) {
          *buffer++ = imgfile->buffer[imgfile->bufptr++];
          --len;
          --bytesleft;
          ++counter;
          ++(imgfile->position);
        }
      } else {
        while (len) {
          *buffer++ = imgfile->buffer[imgfile->bufptr++];
          --len;
          --bytesleft;
          ++counter;
          ++(imgfile->position);
        }
      }
    }
  }
  return counter;
}



void di_close(ImageFile *imgfile) {
  --(imgfile->diskimage->openfiles);
}
