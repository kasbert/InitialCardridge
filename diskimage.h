
#include <SdFat.h>

/* constants for the supported disk formats */
#define MAXTRACKS 80
#define MAXSECTORS 40

#define D64SIZE 174848
#define D64ERRSIZE 175531
#define D71SIZE 349696
#define D71ERRSIZE 351062
#define D81SIZE 819200
#define D81ERRSIZE 822400

typedef enum imagetype {
  D64 = 1,
  D71,
  D81
} ImageType;

typedef enum filetype {
  T_DEL = 0,
  T_SEQ,
  T_PRG,
  T_USR,
  T_REL,
  T_CBM,
  T_DIR
} FileType;

typedef struct ts {
  unsigned char track;
  unsigned char sector;
} TrackSector;

typedef struct diskimage {
  //char *filename;
  long size;
  ImageType type;
  File file;
  TrackSector bam;
  TrackSector bam2;
  TrackSector dir;
  int openfiles;
  int blocksfree;
  int status;
	int interleave;
  TrackSector statusts;
  unsigned char blkbuffer[256];
  int currentblock;
} DiskImage;

typedef struct rawdirentry {
  TrackSector nextts;
  unsigned char type;
  TrackSector startts;
  unsigned char rawname[16];
  TrackSector relsidets;
  unsigned char relrecsize;
  unsigned char unused[4];
  TrackSector replacetemp;
  unsigned char sizelo;
  unsigned char sizehi;
} RawDirEntry;

typedef struct imagefile {
  DiskImage *diskimage;
  RawDirEntry *rawdirentry;
  //char mode;
  int position;
  TrackSector ts;
  TrackSector nextts;
  unsigned char *buffer;
  int bufptr;
  int buflen;
  unsigned char visited[MAXTRACKS][MAXSECTORS/8];
} ImageFile;


bool di_load_image(DiskImage *di, const char *name);
void di_free_image(DiskImage *di);
void di_sync(DiskImage *di);

int di_status(DiskImage *di, char *status);

bool di_open(DiskImage *di, ImageFile *imgfile, unsigned char *rawname, FileType type, const char *mode);
void di_close(ImageFile *imgfile);
int di_read(ImageFile *imgfile, unsigned char *buffer, int len);

unsigned char *di_get_ts_addr(DiskImage *di, TrackSector ts);
int di_get_ts_err(DiskImage *di, TrackSector ts);

int di_sectors_per_track(ImageType type, int track);
int di_tracks(ImageType type);

TrackSector di_get_dir_ts(DiskImage *di);
unsigned char *di_title(DiskImage *di);
int di_track_blocks_free(DiskImage *di, int track);
int di_is_ts_free(DiskImage *di, TrackSector ts);

int di_rawname_from_name(unsigned char *rawname, char *name);
int di_name_from_rawname(char *name, unsigned char *rawname);
