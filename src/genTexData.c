
#include <stdio.h>
#include <string.h>
#include <AR/ar.h>
#include <AR2/config.h>
#include <AR2/imageFormat.h>
#include <AR2/imageSet.h>
#include <AR2/featureSet.h>
#include <AR2/util.h>
#include <KPM/kpm.h>

#include <sys/param.h> // MAXPATHLEN
#include <unistd.h>

#include <time.h> // time(), localtime(), strftime()

#define          KPM_SURF_FEATURE_DENSITY_L0    70
#define          KPM_SURF_FEATURE_DENSITY_L1   100
#define          KPM_SURF_FEATURE_DENSITY_L2   150
#define          KPM_SURF_FEATURE_DENSITY_L3   200

#define          TRACKING_EXTRACTION_LEVEL_DEFAULT 2
#define          INITIALIZATION_EXTRACTION_LEVEL_DEFAULT 1
#define KPM_MINIMUM_IMAGE_SIZE 28 // Filter size for 1 octaves plus 1.
//#define KPM_MINIMUM_IMAGE_SIZE 196 // Filter size for 4 octaves plus 1.

#ifndef MIN
#  define MIN(x,y) (x < y ? x : y)
#endif

enum {
    E_NO_ERROR = 0,
    E_BAD_PARAMETER = 64,
    E_INPUT_DATA_ERROR = 65,
    E_USER_INPUT_CANCELLED = 66,
    E_BACKGROUND_OPERATION_UNSUPPORTED = 69,
    E_DATA_PROCESSING_ERROR = 70,
    E_UNABLE_TO_DETACH_FROM_CONTROLLING_TERMINAL = 71,
    E_GENERIC_ERROR = 255
};

static int                  genfset = 1;
static int                  genfset3 = 1;

static char                 filename[MAXPATHLEN] = "";
static char                 outputFilename[MAXPATHLEN] = "";
static AR2JpegImageT       *jpegImage;
//static ARUint8             *image;
static int                  xsize, ysize;
static int                  nc;
static float                dpi = 220.0f;

static float                dpiMin = 20.0f;
static float                dpiMax = 80.0f;
static float                dpi_list[] = {80.000008, 63.496048, 50.396847, 40.000004, 31.748022, 25.198421, 20.000000};
static int                  dpi_num = 7;

static float                sd_thresh  = -1.0f;
static float                min_thresh = -1.0f;
static float                max_thresh = -1.0f;
static int                  featureDensity = -1;
static int                  occ_size = -1;
static int                  tracking_extraction_level = 0; // Allows specification from command-line.
static int                  initialization_extraction_level = 1;

static char                 exitcodefile[MAXPATHLEN] = "";
static char                 exitcode = 255;
#define EXIT(c) {exitcode=c;exit(c);}


static void  usage( char *com );
static int   readImageFromFile(const char *filename, ARUint8 **image_p, int *xsize_p, int *ysize_p, int *nc_p, float *dpi_p);
//static int   setDPI( void );
static void  write_exitcode(void);

int main( int argc, char *argv[] )
{
    AR2JpegImageT       *jpegImage = NULL;
    ARUint8             *image = NULL;
    AR2ImageSetT        *imageSet = NULL;
    AR2FeatureMapT      *featureMap = NULL;
    AR2FeatureSetT      *featureSet = NULL;
    KpmRefDataSet       *refDataSet = NULL;
    float                scale1, scale2;
    int                  procMode;
    char                 buf[1024];
    int                  num;
    int                  i, j;
    char                *sep = NULL;
	time_t				 clock;
    int                  maxFeatureNum;
    int                  err;

    for( i = 1; i < argc; i++ ) {
        if( strncmp(argv[i], "-dpi=", 5) == 0 ) {
            if( sscanf(&argv[i][5], "%f", &dpi) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-sd_thresh=", 11) == 0 ) {
            if( sscanf(&argv[i][11], "%f", &sd_thresh) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-max_thresh=", 12) == 0 ) {
            if( sscanf(&argv[i][12], "%f", &max_thresh) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-min_thresh=", 12) == 0 ) {
            if( sscanf(&argv[i][12], "%f", &min_thresh) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-feature_density=", 13) == 0 ) {
            if( sscanf(&argv[i][13], "%d", &featureDensity) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-level=", 7) == 0 ) {
            if( sscanf(&argv[i][7], "%d", &tracking_extraction_level) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-leveli=", 8) == 0 ) {
            if( sscanf(&argv[i][8], "%d", &initialization_extraction_level) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-max_dpi=", 9) == 0 ) {
            if( sscanf(&argv[i][9], "%f", &dpiMax) != 1 ) usage(argv[0]);
        } else if( strncmp(argv[i], "-min_dpi=", 9) == 0 ) {
            if( sscanf(&argv[i][9], "%f", &dpiMin) != 1 ) usage(argv[0]);
        } else if( strcmp(argv[i], "-nofset") == 0 ) {
            genfset = 0;
        } else if( strcmp(argv[i], "-fset") == 0 ) {
            genfset = 1;
        } else if( strcmp(argv[i], "-nofset2") == 0 ) {
            ARLOGe("Error: -nofset2 option no longer supported as of ARToolKit v5.3.\n");
            exit(-1);
        } else if( strcmp(argv[i], "-fset2") == 0 ) {
            ARLOGe("Error: -fset2 option no longer supported as of ARToolKit v5.3.\n");
            exit(-1);
        } else if( strcmp(argv[i], "-nofset3") == 0 ) {
            genfset3 = 0;
        } else if( strcmp(argv[i], "-fset3") == 0 ) {
            genfset3 = 1;
        } else if( strncmp(argv[i], "-loglevel=", 10) == 0 ) {
            if (strcmp(&(argv[i][10]), "DEBUG") == 0) arLogLevel = AR_LOG_LEVEL_DEBUG;
            else if (strcmp(&(argv[i][10]), "INFO") == 0) arLogLevel = AR_LOG_LEVEL_INFO;
            else if (strcmp(&(argv[i][10]), "WARN") == 0) arLogLevel = AR_LOG_LEVEL_WARN;
            else if (strcmp(&(argv[i][10]), "ERROR") == 0) arLogLevel = AR_LOG_LEVEL_ERROR;
            else usage(argv[0]);
         } else if( strncmp(argv[i], "-exitcode=", 10) == 0 ) {
            strncpy(exitcodefile, &(argv[i][10]), sizeof(exitcodefile) - 1);
            exitcodefile[sizeof(exitcodefile) - 1] = '\0'; // Ensure NULL termination.
        } else if (strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-version") == 0 || strcmp(argv[i], "-v") == 0) {
            ARLOG("%s version %s\n", argv[0], AR_HEADER_VERSION_STRING);
            exit(0);
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-?") == 0) {
            usage(argv[0]);
        } else if( strncmp(argv[i], "-output_dir=", 12) == 0 ) {
            strncpy(outputFilename, &(argv[i][12]), sizeof(outputFilename) - 1);
            outputFilename[sizeof(outputFilename) - 1] = '\0'; // Ensure NULL termination.
        } else if( filename[0] == '\0' ) {
            strncpy(filename, argv[i], sizeof(filename) - 1);
            filename[sizeof(filename) - 1] = '\0'; // Ensure NULL termination.
        } else {
            ARLOGe("Error: unrecognised option '%s'\n", argv[i]);
            usage(argv[0]);
        }
    }
    
    if ( outputFilename[0] == '\0' )
        strcpy(outputFilename, filename);
    else {
        unsigned long i = strlen(filename) - 1;
        while (i > 0 && filename[i] != '/')
            --i;
        if (filename[i] == '/') ++i;
        
        if (outputFilename[strlen(outputFilename) - 1] != '/') {
            strcat(outputFilename, "/");
        }
        strcat(outputFilename, &filename[i]);
    }
    
    // Do some checks on the input.
    if (filename[0] == '\0') {
        ARLOGe("Error: no input file specified. Exiting.\n");
        usage(argv[0]);
    }
    sep = strrchr(filename, '.');
    if (!sep || (strcmp(sep, ".jpeg") && strcmp(sep, ".jpg") && strcmp(sep, ".jpe") && strcmp(sep, ".JPEG") && strcmp(sep, ".JPE") && strcmp(sep, ".JPG"))) {
        ARLOGe("Error: input file must be a JPEG image (with suffix .jpeg/.jpg/.jpe). Exiting.\n");
        usage(argv[0]);
    }

    if (exitcodefile[0]) {
        atexit(write_exitcode);
    }

    if (genfset) {
        if (tracking_extraction_level == -1 && (sd_thresh  == -1.0 || min_thresh == -1.0 || max_thresh == -1.0 || occ_size == -1)) {
            do {
                printf("Select extraction level for tracking features, 0(few) <--> 4(many), [default=%d]: ", TRACKING_EXTRACTION_LEVEL_DEFAULT);
                if( fgets(buf, sizeof(buf), stdin) == NULL ) EXIT(E_USER_INPUT_CANCELLED);
                if (buf[0] == '\n') tracking_extraction_level = TRACKING_EXTRACTION_LEVEL_DEFAULT;
                else sscanf(buf, "%d", &tracking_extraction_level);
            } while (tracking_extraction_level < 0 || tracking_extraction_level > 4);
        }
        switch (tracking_extraction_level) {
            case 0:
                if( sd_thresh  == -1.0f ) sd_thresh  = AR2_DEFAULT_SD_THRESH_L0;
                if( min_thresh == -1.0f ) min_thresh = AR2_DEFAULT_MIN_SIM_THRESH_L0;
                if( max_thresh == -1.0f ) max_thresh = AR2_DEFAULT_MAX_SIM_THRESH_L0;
                if( occ_size   == -1    ) occ_size   = AR2_DEFAULT_OCCUPANCY_SIZE;
                break;
            case 1:
                if( sd_thresh  == -1.0f ) sd_thresh  = AR2_DEFAULT_SD_THRESH_L1;
                if( min_thresh == -1.0f ) min_thresh = AR2_DEFAULT_MIN_SIM_THRESH_L1;
                if( max_thresh == -1.0f ) max_thresh = AR2_DEFAULT_MAX_SIM_THRESH_L1;
                if( occ_size   == -1    ) occ_size   = AR2_DEFAULT_OCCUPANCY_SIZE;
                break;
            case 2:
                if( sd_thresh  == -1.0f ) sd_thresh  = AR2_DEFAULT_SD_THRESH_L2;
                if( min_thresh == -1.0f ) min_thresh = AR2_DEFAULT_MIN_SIM_THRESH_L2;
                if( max_thresh == -1.0f ) max_thresh = AR2_DEFAULT_MAX_SIM_THRESH_L2;
                if( occ_size   == -1    ) occ_size   = AR2_DEFAULT_OCCUPANCY_SIZE*2/3;
                break;
            case 3:
                if( sd_thresh  == -1.0f ) sd_thresh  = AR2_DEFAULT_SD_THRESH_L3;
                if( min_thresh == -1.0f ) min_thresh = AR2_DEFAULT_MIN_SIM_THRESH_L3;
                if( max_thresh == -1.0f ) max_thresh = AR2_DEFAULT_MAX_SIM_THRESH_L3;
                if( occ_size   == -1    ) occ_size   = AR2_DEFAULT_OCCUPANCY_SIZE*2/3;
                break;
            case 4: // Same as 3, but with smaller AR2_DEFAULT_OCCUPANCY_SIZE.
                if( sd_thresh  == -1.0f ) sd_thresh  = AR2_DEFAULT_SD_THRESH_L3;
                if( min_thresh == -1.0f ) min_thresh = AR2_DEFAULT_MIN_SIM_THRESH_L3;
                if( max_thresh == -1.0f ) max_thresh = AR2_DEFAULT_MAX_SIM_THRESH_L3;
                if( occ_size   == -1    ) occ_size   = AR2_DEFAULT_OCCUPANCY_SIZE*1/2;
                break;
             default: // We only get to here if the parameters are already set.
                break;
        }
        ARLOGi("MAX_THRESH  = %f\n", max_thresh);
        ARLOGi("MIN_THRESH  = %f\n", min_thresh);
        ARLOGi("SD_THRESH   = %f\n", sd_thresh);
    }
    if (genfset3) {
        if (initialization_extraction_level == -1 && featureDensity == -1) {
            do {
                printf("Select extraction level for initializing features, 0(few) <--> 3(many), [default=%d]: ", INITIALIZATION_EXTRACTION_LEVEL_DEFAULT);
                if( fgets(buf,1024,stdin) == NULL ) EXIT(E_USER_INPUT_CANCELLED);
                if (buf[0] == '\n') initialization_extraction_level = INITIALIZATION_EXTRACTION_LEVEL_DEFAULT;
                else sscanf(buf, "%d", &initialization_extraction_level);
            } while (initialization_extraction_level < 0 || initialization_extraction_level > 3);
        }
        switch(initialization_extraction_level) {
            case 0:
                if( featureDensity  == -1 ) featureDensity  = KPM_SURF_FEATURE_DENSITY_L0;
                break;
            default:
            case 1:
                if( featureDensity  == -1 ) featureDensity  = KPM_SURF_FEATURE_DENSITY_L1;
                break;
            case 2:
                if( featureDensity  == -1 ) featureDensity  = KPM_SURF_FEATURE_DENSITY_L2;
                break;
            case 3:
                if( featureDensity  == -1 ) featureDensity  = KPM_SURF_FEATURE_DENSITY_L3;
                break;
        }
        ARLOGi("SURF_FEATURE = %d\n", featureDensity);
    }

    if ((err = readImageFromFile(filename, &image, &xsize, &ysize, &nc, &dpi)) != 0) {
        ARLOGe("Error reading image from file '%s'.\n", filename);
        EXIT(err);
    }

    //setDPI();

    ARLOGi("Generating ImageSet...\n");
    ARLOGi("   (Source image xsize=%d, ysize=%d, channels=%d, dpi=%.1f).\n", xsize, ysize, nc, dpi);
    imageSet = ar2GenImageScale0( image, xsize, ysize, nc, dpi, dpi_list, dpi_num );
    ar2FreeJpegImage(&jpegImage);
    if( imageSet == NULL ) {
        ARLOGe("ImageSet generation error!!\n");
        EXIT(E_DATA_PROCESSING_ERROR);
    }
    ARLOGi("  Done.\n");
    ar2UtilRemoveExt( outputFilename );
    ARLOGi("Saving to %s.iset...\n", filename);
    FILE *imageSetFP;
    if( (imageSetFP = ar2WriteImageScale0(outputFilename, imageSet)) == NULL) {
        ARLOGe("Save error: %s.iset\n", outputFilename );
        EXIT(E_DATA_PROCESSING_ERROR);
    }
    ARLOGi("  Done.\n");

    if (genfset && genfset3)
    {
        arMalloc( featureSet, AR2FeatureSetT, 1 );                      // A featureSet with a single image,
        arMalloc( featureSet->list, AR2FeaturePointsT, imageSet->num ); // and with 'num' scale levels of this image.
        featureSet->num = imageSet->num;

        refDataSet  = NULL;
        procMode    = KpmProcFullSize;
        
        ARLOGi("Generating FeatureList...\n");
        ARLOGi("Generating FeatureSet3...\n");
        for( i = 0; i < imageSet->num; i++ ) {
            if (i > 0) {
                imageSet->scale[i] = ar2GenImageLayer2( imageSet->scale[0], dpi_list[i] );
                if (i > 1) {
                    free(imageSet->scale[i - 1]->imgBW);
                    free(imageSet->scale[i - 1]);
                    imageSet->scale[i - 1] = NULL;
                }
         
                if (i == (imageSet->num - 1)) {
                    free(imageSet->scale[0]->imgBW);
                    free(imageSet->scale[0]);
                    imageSet->scale[0] = NULL;
                }
                
                if( fwrite(&(imageSet->scale[i]->dpi), sizeof(imageSet->scale[i]->dpi), 1, imageSetFP) != 1 ) {
                    ARLOGe("Save error: %s.iset\n", outputFilename );
                    EXIT(E_DATA_PROCESSING_ERROR);
                }
            }

            ARLOGi("Start for %f dpi image.\n", imageSet->scale[i]->dpi);
            
            featureMap = ar2GenFeatureMap( imageSet->scale[i],
                                          AR2_DEFAULT_TS1*AR2_TEMP_SCALE, AR2_DEFAULT_TS2*AR2_TEMP_SCALE,
                                          AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE1, AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE2,
                                          AR2_DEFAULT_MAX_SIM_THRESH2, AR2_DEFAULT_SD_THRESH2 );
            if( featureMap == NULL ) {
                ARLOGe("Error!!\n");
                EXIT(E_DATA_PROCESSING_ERROR);
            }
            ARLOGi("  Done.\n");
            
            
            featureSet->list[i].coord = ar2SelectFeature2( imageSet->scale[i], featureMap,
                                                          AR2_DEFAULT_TS1*AR2_TEMP_SCALE, AR2_DEFAULT_TS2*AR2_TEMP_SCALE, AR2_DEFAULT_GEN_FEATURE_MAP_SEARCH_SIZE2,
                                                          occ_size,
                                                          max_thresh, min_thresh, sd_thresh, &num );
            if( featureSet->list[i].coord == NULL ) num = 0;
            featureSet->list[i].num   = num;
            featureSet->list[i].scale = i;

            if (i < imageSet->num - 1)
                scale1 = dpi_list[i + 1];
            else scale1 = 0.0f;

            if( scale1 == 0.0f ) {
                featureSet->list[i].mindpi = imageSet->scale[i]->dpi * 0.5f;
            }
            else {
                featureSet->list[i].mindpi = scale1;
            }
            
            if (i > 0)
                scale1 = dpi_list[i - 1];
            else scale1 = 0.0f;

            if( scale1 == 0.0f ) {
                featureSet->list[i].maxdpi = imageSet->scale[i]->dpi * 2.0f;
            }
            else {
                scale2 = imageSet->scale[i]->dpi;
                featureSet->list[i].maxdpi = scale2*0.8f + scale1*0.2f;
            }
            
            ar2FreeFeatureMap( featureMap );
            
            maxFeatureNum = featureDensity * imageSet->scale[i]->xsize * imageSet->scale[i]->ysize / (480*360);
            ARLOGi("(%d, %d) %f[dpi]\n", imageSet->scale[i]->xsize, imageSet->scale[i]->ysize, imageSet->scale[i]->dpi);
            if( kpmAddRefDataSet (
                                  imageSet->scale[i]->imgBW,
                                  AR_PIXEL_FORMAT_MONO,
                                  imageSet->scale[i]->xsize,
                                  imageSet->scale[i]->ysize,
                                  imageSet->scale[i]->dpi,
                                  procMode, KpmCompNull, maxFeatureNum, 1, i, &refDataSet) < 0 ) { // Page number set to 1 by default.
                ARLOGe("Error at kpmAddRefDataSet.\n");
                EXIT(E_DATA_PROCESSING_ERROR);
            }
        }
        ARLOGi("  Done.\n");
        
        ARLOGi("Saving FeatureSet...\n");
        if( ar2SaveFeatureSet( outputFilename, "fset", featureSet ) < 0 ) {
            ARLOGe("Save error: %s.fset\n", outputFilename );
            EXIT(E_DATA_PROCESSING_ERROR);
        }
        ARLOGi("  Done.\n");
        ar2FreeFeatureSet( &featureSet );

        ARLOGi("Saving FeatureSet3...\n");
        if( kpmSaveRefDataSet(outputFilename, "fset3", refDataSet) != 0 ) {
            ARLOGe("Save error: %s.fset2\n", outputFilename );
            EXIT(E_DATA_PROCESSING_ERROR);
        }
        ARLOGi("  Done.\n");
        kpmDeleteRefDataSet( &refDataSet );
    }
    
    fclose(imageSetFP);

    ar2FreeImageSet( &imageSet );

    exitcode = E_NO_ERROR;
    return (exitcode);
}

static void usage( char *com )
{
        ARLOG("%s <filename>\n", com);
        ARLOG("    -level=n\n"
              "         (n is an integer in range 0 (few) to 4 (many). Default %d.'\n", TRACKING_EXTRACTION_LEVEL_DEFAULT);
        ARLOG("    -sd_thresh=<sd_thresh>\n");
        ARLOG("    -max_thresh=<max_thresh>\n");
        ARLOG("    -min_thresh=<min_thresh>\n");
        ARLOG("    -leveli=n\n"
              "         (n is an integer in range 0 (few) to 3 (many). Default %d.'\n", INITIALIZATION_EXTRACTION_LEVEL_DEFAULT);
        ARLOG("    -feature_density=<feature_density>\n");
        ARLOG("    -dpi=f: Override embedded JPEG DPI value.\n");
        ARLOG("    -max_dpi=<max_dpi>\n");
        ARLOG("    -min_dpi=<min_dpi>\n");
        ARLOG("    -background\n");
        ARLOG("         Run in background, i.e. as daemon detached from controlling terminal. (Mac OS X and Linux only.)\n");
        ARLOG("    -log=<path>\n");
        ARLOG("    -loglevel=x\n");
        ARLOG("         x is one of: DEBUG, INFO, WARN, ERROR. Default is %s.\n", (AR_LOG_LEVEL_DEFAULT == AR_LOG_LEVEL_DEBUG ? "DEBUG" : (AR_LOG_LEVEL_DEFAULT == AR_LOG_LEVEL_INFO ? "INFO" : (AR_LOG_LEVEL_DEFAULT == AR_LOG_LEVEL_WARN ? "WARN" : (AR_LOG_LEVEL_DEFAULT == AR_LOG_LEVEL_ERROR ? "ERROR" : "UNKNOWN")))));
        ARLOG("    -exitcode=<path>\n");
        ARLOG("    --help -h -?  Display this help\n");

    EXIT(E_BAD_PARAMETER);
}

static int readImageFromFile(const char *filename, ARUint8 **image_p, int *xsize_p, int *ysize_p, int *nc_p, float *dpi_p)
{
    char *ext;
    char buf[256];
    char buf1[512], buf2[512];
    
    if (!filename || !image_p || !xsize_p || !ysize_p || !nc_p || !dpi_p) return (E_BAD_PARAMETER);

    ext = arUtilGetFileExtensionFromPath(filename, 1);
    if (!ext) {
        ARLOGe("Error: unable to determine extension of file '%s'. Exiting.\n", filename);
        EXIT(E_INPUT_DATA_ERROR);
    }
    if (strcmp(ext, "jpeg") == 0 || strcmp(ext, "jpg") == 0 || strcmp(ext, "jpe") == 0) {
        
        ARLOGi("Reading JPEG file...\n");
        ar2UtilDivideExt( filename, buf1, buf2 );
        jpegImage = ar2ReadJpegImage( buf1, buf2 );
        if( jpegImage == NULL ) {
            ARLOGe("Error: unable to read JPEG image from file '%s'. Exiting.\n", filename);
            EXIT(E_INPUT_DATA_ERROR);
        }
        ARLOGi("   Done.\n");
        
        *image_p = jpegImage->image;
        if (jpegImage->nc != 1 && jpegImage->nc != 3) {
            ARLOGe("Error: 2 byte/pixel JPEG files not currently supported. Exiting.\n");
            EXIT(E_INPUT_DATA_ERROR);
        }
        *nc_p    = jpegImage->nc;
        ARLOGi("JPEG image '%s' is %dx%d.\n", filename, jpegImage->xsize, jpegImage->ysize);
        if (jpegImage->xsize < KPM_MINIMUM_IMAGE_SIZE || jpegImage->ysize < KPM_MINIMUM_IMAGE_SIZE) {
            ARLOGe("Error: JPEG image width and height must be at least %d pixels. Exiting.\n", KPM_MINIMUM_IMAGE_SIZE);
            EXIT(E_INPUT_DATA_ERROR);
        }
        *xsize_p = jpegImage->xsize;
        *ysize_p = jpegImage->ysize;
        if (*dpi_p == -1.0) {
            if( jpegImage->dpi == 0.0f ) {
                for (;;) {
                    printf("JPEG image '%s' does not contain embedded resolution data, and no resolution specified on command-line.\nEnter resolution to use (in decimal DPI): ", filename);
                    if( fgets( buf, 256, stdin ) == NULL ) {
                        EXIT(E_USER_INPUT_CANCELLED);
                    }
                    if( sscanf(buf, "%f", &(jpegImage->dpi)) == 1 ) break;
                }
            }
            *dpi_p   = jpegImage->dpi;
        }

    //} else if (strcmp(ext, "png") == 0) {
        
        
        
    } else {
        ARLOGe("Error: file '%s' has extension '%s', which is not supported for reading. Exiting.\n", filename, ext);
        free(ext);
        EXIT(E_INPUT_DATA_ERROR);
    }
    free(ext);
    
    return 0;
}

static void write_exitcode(void)
{
    if (exitcodefile[0]) {
        FILE *fp = fopen(exitcodefile, "w");
        fprintf(fp, "%d\n", exitcode);
        fclose(fp);
    }
}

