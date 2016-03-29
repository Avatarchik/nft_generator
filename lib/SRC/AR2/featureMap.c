
#include <AR/ar.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <AR2/config.h>
#include <AR2/featureSet.h>

static float *fimage2;

static int cmp(const void *p, const void *q) {
    int l = *(const int *)p;
    int r = *(const int *)q;
    
    if (fimage2[l] < fimage2[r]) return -1;
    if (fimage2[l] == fimage2[r]) return 0;
    
    return 1;
}

static int make_template( ARUint8 *imageBW, int xsize, int ysize,
                         int cx, int cy, int ts1, int ts2, float  sd_thresh,
                         float  *template, float  *vlen, float *vlen2s);

static int get_similarity( ARUint8 *imageBW, int xsize, int ysize,
                          float  *template, float  vlen, int ts1, int ts2,
                          int cx, int cy, float  *sim, float *vlen2s);

static int make_template_origin( ARUint8 *imageBW, int xsize, int ysize,
                                int cx, int cy, int ts1, int ts2, float  sd_thresh,
                                float  *template, float  *vlen );

static int get_similarity_origin( ARUint8 *imageBW, int xsize, int ysize,
                                 float  *template, float  vlen, int ts1, int ts2,
                                 int cx, int cy, float  *sim);

int ar2FreeFeatureMap( AR2FeatureMapT *featureMap )
{
    free( featureMap->map );
    free( featureMap);
    
    return 0;
}

int ar2SaveFeatureMap( char *filename, char *ext, AR2FeatureMapT *featureMap )
{
    FILE   *fp;
    
    char buf[512];
    sprintf(buf, "%s.%s", filename, ext);
    if( (fp=fopen(buf, "wb")) == NULL ) return -1;
    
    if( fwrite(&(featureMap->xsize), sizeof(featureMap->xsize), 1, fp) != 1 ) goto bailBadWrite;
    if( fwrite(&(featureMap->ysize), sizeof(featureMap->ysize), 1, fp) != 1 ) goto bailBadWrite;
    if( fwrite(featureMap->map, sizeof(float), (featureMap->xsize)*(featureMap->ysize), fp) != (featureMap->xsize)*(featureMap->ysize) ) goto bailBadWrite;
    
    fclose(fp);
    return 0;
    
bailBadWrite:
    ARLOGe("Error saving feature map: error writing data.\n");
    fclose(fp);
    return (-1);
}

AR2FeatureMapT *ar2ReadFeatureMap( char *filename, char *ext )
{
    AR2FeatureMapT  *featureMap;
    FILE            *fp;
    
    char buf[512];
    sprintf(buf, "%s.%s", filename, ext);
    if( (fp=fopen(buf, "rb")) == NULL ) return NULL;
    
    arMalloc( featureMap, AR2FeatureMapT, 1 );
    
    if( fread(&(featureMap->xsize), sizeof(featureMap->xsize), 1, fp) != 1 ) {
        fclose(fp);
        free(featureMap);
        return NULL;
    }
    
    if( fread(&(featureMap->ysize), sizeof(featureMap->ysize), 1, fp) != 1 ) {
        fclose(fp);
        free(featureMap);
        return NULL;
    }
    
    arMalloc( featureMap->map, float, (featureMap->xsize)*(featureMap->ysize) );
    
    if( fread(featureMap->map, sizeof(float), (featureMap->xsize)*(featureMap->ysize), fp) != (featureMap->xsize)*(featureMap->ysize) ) {
        free(featureMap->map);
        free(featureMap);
        fclose(fp);
        return NULL;
    }
    
    fclose(fp);
    
    return featureMap;
}

AR2FeatureMapT *ar2GenFeatureMap( AR2ImageT *image,
                                 int ts1, int ts2,
                                 int search_size1, int search_size2,
                                 float  max_sim_thresh, float  sd_thresh )
{
    AR2FeatureMapT  *featureMap;
    float           *fimage, *fp;
    float           *fimage2, *fp2;
    float           *template;
    ARUint8         *p;
    float           dx, dy;
    int             xsize, ysize;
    int             hist[1000], sum;
    int             i, j, k;
    float           vlen;
    float           max, sim;
    int             ii, jj;
    float           *vlen2s;
    
    xsize = image->xsize;
    ysize = image->ysize;
    arMalloc(fimage,   float,  xsize*ysize);
    arMalloc(fimage2,  float,  xsize*ysize);
    arMalloc(template, float , (ts1+ts2+1)*(ts1+ts2+1));
    
    arMalloc(vlen2s,   float,  xsize*ysize);
    memset(vlen2s, 0, xsize*ysize * sizeof(float));
    
    fp2 = fimage2;
    p = image->imgBW;
    
    for( i = 0; i < xsize; i++ ) {*(fp2++) = -1.0f; p++;}
    for( j = 1; j < ysize-1; j++ ) {
        *(fp2++) = -1.0f; p++;
        for( i = 1; i < xsize-1; i++ ) {
            dx = ((int)(*(p-xsize+1)) - (int)(*(p-xsize-1))
                  +  (int)(*(p      +1)) - (int)(*(p      -1))
                  +  (int)(*(p+xsize+1)) - (int)(*(p+xsize-1))) / (float )(3.0f*256);
            dy = ((int)(*(p+xsize+1)) - (int)(*(p-xsize+1))
                  +  (int)(*(p+xsize  )) - (int)(*(p-xsize  ))
                  +  (int)(*(p+xsize-1)) - (int)(*(p-xsize-1))) / (float )(3.0f*256);
            *(fp2++) = (float)sqrtf( (dx*dx+dy*dy) / (float )2.0f );
            p++;
        }
        *(fp2++) = -1.0f; p++;
    }
    for( i = 0; i < xsize; i++ ) {*(fp2++) = -1.0f; p++;}
    
    
    sum = 0;
    for( i = 0; i < 1000; i++ ) hist[i] = 0;
    fp2 = fimage2 + xsize + 1;
    for( j = 1; j < ysize-1; j++ ) {
        for( i = 1; i < xsize-1; i++ ) {
            if( *fp2 > *(fp2-1) && *fp2 > *(fp2+1) && *fp2 > *(fp2-xsize) && *fp2 > *(fp2+xsize) ) {
                k = (int)(*fp2 * 1000.0f);
                if( k > 999 ) k = 999;
                if( k < 0   ) k = 0;
                hist[k]++;
                sum++;
            }
            fp2++;
        }
        fp2 += 2;
    }
    j = 0;
    for( i = 999; i >= 0; i-- ) {
        j += hist[i];
        if( (float )j / (float )(xsize*ysize) >= 0.02f ) break;
        //if( (float )j / (float )(xsize*ysize) >= 0.2f ) break;
    }
    k = i;
    ARLOGi("ar2GenFeatureMap          ImageSize = %7d[pixel]\n", xsize*ysize);
    ARLOGi("ar2GenFeatureMap Extracted features = %7d[pixel]\n", sum);
    ARLOGi("ar2GenFeatureMap  Filtered features = %7d[pixel]\n", j);
    
    
    fp = fimage;
    fp2 = fimage2;
    for( i = 0; i < xsize; i++ ) {
        *(fp++) = 1.0f;
        fp2++;
    }
    for( j = 1; j < ysize-1; j++ ) {
        ARLOGi("\r%4d/%4d.", j+1, ysize);
        *(fp++) = 1.0f;
        fp2++;
        for( i = 1; i < xsize-1; i++ ) {
            if( *fp2 <= *(fp2-1) || *fp2 <= *(fp2+1) || *fp2 <= *(fp2-xsize) || *fp2 <= *(fp2+xsize) ) {
                *(fp++) = 1.0f;
                fp2++;
                continue;
            }
            if( (int)(*fp2 * 1000) < k ) {
                *(fp++) = 1.0f;
                fp2++;
                continue;
            }
            if( make_template(image->imgBW, xsize, ysize, i, j, ts1, ts2, sd_thresh, template, &vlen, vlen2s) < 0 ) {
                *(fp++) = 1.0f;
                fp2++;
                continue;
            }
            
            max = -1.0f;
            for( jj = -search_size1; jj <= search_size1; jj++ ) {
                for( ii = -search_size1; ii <= search_size1; ii++ ) {
                    
                    if( ii*ii + jj*jj <= search_size2*search_size2 ) continue;
                    //if( jj >= -search_size2 && jj <= search_size2 && ii >= -search_size2 && ii <= search_size2 ) continue;
                    
                    if( get_similarity(image->imgBW, xsize, ysize, template, vlen, ts1, ts2, i+ii, j+jj, &sim, vlen2s) < 0 ) continue;
                    
                    if( sim > max ) {
                        max = sim;
                        if( max > max_sim_thresh ) break;
                    }
                }
                if( max > max_sim_thresh ) break;
            }
            *(fp++) = (float)max;
            fp2++;
        }
        *(fp++) = 1.0f;
        fp2++;
    }
    ARLOGi("\n");
    free(fimage2);
    free(template);
    free(vlen2s);
    
    arMalloc( featureMap, AR2FeatureMapT, 1 );
    featureMap->map = fimage;
    featureMap->xsize = xsize;
    featureMap->ysize = ysize;
    
    return featureMap;
}


AR2FeatureCoordT *ar2SelectFeature( AR2ImageT *image, AR2FeatureMapT *featureMap,
                                   int ts1, int ts2, int search_size2, int occ_size,
                                   float  max_sim_thresh, float  min_sim_thresh, float  sd_thresh, int *num )
{
    AR2FeatureCoordT   *coord;
    float              *template, vlen;
    float              *fimage2, *fp1, *fp2;
    float              min_sim;
    float              sim, min, max;
    float              dpi;
    int                xsize, ysize;
    int                max_feature_num;
    int                cx, cy;
    int                i, j;
    float              *vlen2s;
    
    if( image->xsize != featureMap->xsize || image->ysize != featureMap->ysize ) return NULL;
    
    xsize = image->xsize;
    ysize = image->ysize;
    dpi = image->dpi;
    arMalloc(template, float , (ts1+ts2+1)*(ts1+ts2+1));
    arMalloc(fimage2, float, xsize*ysize);
    arMalloc(vlen2s,   float,  xsize*ysize);
    memset(vlen2s, 0, xsize*ysize * sizeof(float));
    
    fp1 = featureMap->map;
    fp2 = fimage2;
    for( i = 0; i < xsize*ysize; i++ ) {
        *(fp2++) = *(fp1++);
    }
    
    max_feature_num = (xsize/occ_size)*(ysize/occ_size);
    if( max_feature_num < 10 ) max_feature_num = 10;
    ARLOGi("ar2SelectFeature: Max feature = %d\n", max_feature_num);
    arMalloc( coord, AR2FeatureCoordT, max_feature_num );
    *num = 0;
    
    while( *num < max_feature_num ) {
        
        min_sim = max_sim_thresh;
        fp2 = fimage2;
        cx = cy = -1;
        for( j = 0; j < ysize; j++ ) {
            for( i = 0; i < xsize; i++ ) {
                if( *fp2 < min_sim ) {
                    min_sim = *fp2;
                    cx = i;
                    cy = j;
                }
                fp2++;
            }
        }
        if( cx == -1 ) break;
        
        
        if( make_template( image->imgBW, xsize, ysize, cx, cy, ts1, ts2, 0.0, template, &vlen, vlen2s) < 0 ) {
            
            fimage2[cy*xsize+cx] = 1.0f;
            continue;
        }
        if( vlen/(ts1+ts2+1) < sd_thresh ) {
            fimage2[cy*xsize+cx] = 1.0f;
            continue;
        }
        
        min = 1.0f;
        max = -1.0f;
        for( j = -search_size2; j <= search_size2; j++ ) {
            for( i = -search_size2; i <= search_size2; i++ ) {
                if( i*i + j*j > search_size2*search_size2 ) continue;
                if( i == 0 && j == 0 ) continue;
                
                if( get_similarity(image->imgBW, xsize, ysize, template, vlen, ts1, ts2, cx+i, cy+j, &sim, vlen2s) < 0 ) continue;
                
                if( sim < min ) {
                    min = sim;
                    if( min < min_sim_thresh && min < min_sim ) break;
                }
                if( sim > max ) {
                    max = sim;
                    if( max > 0.99f ) break;
                }
            }
            if( (min < min_sim_thresh && min < min_sim) || max > 0.99f ) break;
        }
        
        if( (min < min_sim_thresh && min < min_sim) || max > 0.99f ) {
            fimage2[cy*xsize+cx] = 1.0f;
            continue;
        }
        
        coord[*num].x = cx;
        coord[*num].y = cy;
        coord[*num].mx = (float) cx          / dpi * 25.4f; // millimetres.
        coord[*num].my = (float)(ysize - cy) / dpi * 25.4f; // millimetres.
        coord[*num].maxSim = (float)min_sim;
        (*num)++;
        
        ARLOGi("%3d: (%3d,%3d) : %f min=%f max=%f, sd=%f\n", *num, cx, cy, min_sim, min, max, vlen/(ts1+ts2+1));
        for( j = -occ_size; j <= occ_size; j++ ) {
            for( i = -occ_size; i <= occ_size; i++ ) {
                if( cy+j < 0 || cy+j >= ysize || cx+i < 0 || cx+i >= xsize ) continue;
                
                fimage2[(cy+j)*xsize+(cx+i)] = 1.0f;
            }
        }
    }
    
    free( template );
    free( fimage2 );
    free(vlen2s);
    
    return coord;
}


AR2FeatureCoordT *ar2SelectFeature2( AR2ImageT *image, AR2FeatureMapT *featureMap,
                                    int ts1, int ts2, int search_size2, int occ_size,
                                    float  max_sim_thresh, float  min_sim_thresh, float  sd_thresh, int *num )
{
    AR2FeatureCoordT   *coord;
    float              *template, vlen;
    float              min_sim;
    float              sim, min, max;
    float              *fp1, *fp2;
    float              dpi;
    int                xdiv, ydiv, div_size;
    int                xsize, ysize;
    int                max_feature_num;
    int                cx, cy;
    int                i, j;
    int                ii;
    int                *v;
    float              *vlen2s;
    
    if( image->xsize != featureMap->xsize || image->ysize != featureMap->ysize ) return NULL;
    
    occ_size *= 2;
    
    xsize = image->xsize;
    ysize = image->ysize;
    dpi = image->dpi;
    arMalloc(template, float , (ts1+ts2+1)*(ts1+ts2+1));
    arMalloc(fimage2, float, xsize*ysize);
    arMalloc(v, int, xsize*ysize);
    
    fp1 = featureMap->map;
    fp2 = fimage2;
    for( i = 0; i < xsize*ysize; i++ ) {
        v[i] = i;
        *(fp2++) = *(fp1++);
    }
    
    qsort(v, xsize * ysize, sizeof(int), cmp);
    
    div_size = (ts1+ts2+1)*3;
    xdiv = xsize/div_size;
    ydiv = ysize/div_size;
    
    max_feature_num = (xsize/occ_size)*(ysize/occ_size) + xdiv*ydiv;
    ARLOGi("ar2SelectFeature2: Max feature = %d\n", max_feature_num);
    arMalloc( coord, AR2FeatureCoordT, max_feature_num );
    *num = 0;
    int pos = 0;
    
    while( *num < max_feature_num ) {
        cx = cy = -1;
        while (pos < xsize * ysize && fimage2[v[pos]] >= max_sim_thresh) ++pos;
        
        if (pos < xsize * ysize) {
            int ind = v[pos];
            min_sim = fimage2[ind];
            cx = ind % xsize;
            cy = ind / xsize;
        }
        
        if( cx == -1 ) break;
        
        if( make_template_origin( image->imgBW, xsize, ysize, cx, cy, ts1, ts2, 0.0, template, &vlen) < 0 ) {
            fimage2[cy*xsize+cx] = 1.0f;
            continue;
        }
        if( vlen/(ts1+ts2+1) < sd_thresh ) {
            fimage2[cy*xsize+cx] = 1.0f;
            continue;
        }
        
        min = 1.0f;
        max = -1.0f;
        for( j = -search_size2; j <= search_size2; j++ ) {
            for( i = -search_size2; i <= search_size2; i++ ) {
                if( i*i + j*j > search_size2*search_size2 ) continue;
                if( i == 0 && j == 0 ) continue;
                
                if( get_similarity_origin(image->imgBW, xsize, ysize, template, vlen, ts1, ts2, cx+i, cy+j, &sim) < 0 ) continue;
                
                if( sim < min ) {
                    min = sim;
                    if( min < min_sim_thresh && min < min_sim ) break;
                }
                if( sim > max ) {
                    max = sim;
                    if( max > 0.99f ) break;
                }
            }
            if( (min < min_sim_thresh && min < min_sim) || max > 0.99f ) break;
        }
        
        if( (min < min_sim_thresh && min < min_sim) || max > 0.99f ) {
            fimage2[cy*xsize+cx] = 1.0f;
            continue;
        }
        
        coord[*num].x = cx;
        coord[*num].y = cy;
        coord[*num].mx = (float) cx          / dpi * 25.4f;
        coord[*num].my = (float)(ysize - cy) / dpi * 25.4f;
        coord[*num].maxSim = (float)min_sim;
        (*num)++;
        
        ARLOGi("%3d: (%3d,%3d) : %f min=%f max=%f, sd=%f\n", *num, cx, cy, min_sim, min, max, vlen/(ts1+ts2+1));
        for( j = -occ_size; j <= occ_size; j++ ) {
            for( i = -occ_size; i <= occ_size; i++ ) {
                if( cy+j < 0 || cy+j >= ysize || cx+i < 0 || cx+i >= xsize ) continue;
                
                fimage2[(cy+j)*xsize+(cx+i)] = 1.0f;
            }
        }
    }
    
    free( template );
    free( fimage2 );
    free ( v );
    
    return coord;
}


int ar2PrintFeatureInfo( AR2ImageT *image, AR2FeatureMapT *featureMap, int ts1, int ts2, int search_size2, int cx, int cy )
{
    float       *template, vlen;
    float       max, min, sim;
    int         xsize, ysize;
    int         i, j;
    float       *vlen2s;
    
    if( image->xsize != featureMap->xsize || image->ysize != featureMap->ysize ) return -1;
    
    arMalloc(template, float, (ts1+ts2+1)*(ts1+ts2+1));
    xsize = image->xsize;
    ysize = image->ysize;
    if( cx < 0 || cy < 0 || cx >= xsize || cy >= ysize ) {
        free(template);
        return -1;
    }
    
    if( featureMap->map[cy*xsize+cx] == 1.0 ) {
        ARLOG("%3d, %3d: max_sim = %f\n", cx, cy, featureMap->map[cy*xsize+cx]);
        free( template );
        return 0;
    }
    
    arMalloc(vlen2s, float, xsize*xsize);
    memset(vlen2s, 0, xsize*ysize * sizeof(float));
    
    if( make_template( image->imgBW, xsize, ysize, cx, cy, ts1, ts2, 0.0, template, &vlen, vlen2s) < 0 ) {
        free(vlen2s);
        free( template );
        return -1;
    }
    
    min = 1.0f;
    max = -1.0f;
    ARLOG("\n");
    for( j = -search_size2; j <= search_size2; j++ ) {
        for( i = -search_size2; i <= search_size2; i++ ) {
            if( get_similarity(image->imgBW, xsize, ysize, template, vlen, ts1, ts2, cx+i, cy+j, &sim, vlen2s) < 0 ) continue;
            
            if( (i*i + j*j <= search_size2*search_size2)
               && (i != 0 || j != 0) ) {
                if( sim < min ) min = sim;
                if( sim > max ) max = sim;
            }
            ARLOG("%7.4f ", sim);
        }
        ARLOG("\n");
    }
    ARLOG("\n");
    
    ARLOG("%3d, %3d: max_sim = %f, (max,min) = %f, %f, sd = %f\n", cx, cy, featureMap->map[cy*xsize+cx], max, min, vlen/(ts1+ts2+1));
    free( template );
    free(vlen2s);
    return 0;
}


static int make_template( ARUint8 *imageBW, int xsize, int ysize,
                         int cx, int cy, int ts1, int ts2, float  sd_thresh,
                         float *template, float *vlen, float *vlen2s)
{
    ARUint8  *ip;
    float    *tp;
    float     vlen1, ave;
    int       i, j;
    
    if( cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize ) return -1;
    
    ave = 0.0f;
    
    int pos = (cy-ts1)*xsize+(cx-ts1);
    int ts = ts1 + ts2 + 1;
    
# pragma omp parallel shared ( ts, imageBW ) private ( j )
    for(j = 0; j < ts; ++j) {
        ip = &imageBW[pos + j * xsize];
        
# pragma omp parallel shared ( ts, ip ) private ( i )
# pragma omp for reduction ( + : ave )
        for(i = 0; i < ts ; ++i)
            ave += ip[i];
    }
    ave /= ts * ts;
    
    tp = template;
    vlen1 = 0.0f;
    
    if (vlen2s[pos] == 0) {
        float sum = 0, squaresum = 0;
        
        for( j = 0; j < ts; ++j ) {
            ip = &imageBW[pos + j * xsize];
            for( i = 0; i < ts; ++i ) {
                sum += ip[i];
                squaresum += ip[i] * ip[i];
                *tp = (float )ip[i] - ave;
                vlen1 += *tp * *tp;
                tp++;
            }
        }
        float vlen2 = squaresum - sum*sum/(ts * ts);
        if( vlen2 == 0.0f ) {
            vlen2 = -1;
        } else {
            vlen2 = sqrtf(vlen2);
        }
        vlen2s[pos] = vlen2;
    } else {
        for( j = 0; j < ts; ++j ) {
            ip = &imageBW[pos + j * xsize];
            for( i = 0; i < ts; ++i ) {
                *tp = (float)ip[i] - ave;
                vlen1 += *tp * *tp;
                tp++;
            }
        }
    }
    
    if( fabsf(vlen1) < 0.001 ) return -1;
    if( vlen1/(ts * ts) < sd_thresh*sd_thresh ) return -1;
    
    *vlen = sqrtf(vlen1);
    
    return 0;
}

static int get_similarity( ARUint8 *imageBW, int xsize, int ysize,
                          float *template, float vlen, int ts1, int ts2,
                          int cx, int cy, float *sim, float *vlen2s)
{
    ARUint8   *ip;
    float     *tp, *ttp;
    float     sx, sxx, sxy;
    float     vlen2;
    int       i, j;
    
    if( cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize ) return -1;
    
    tp = template;
    sx = sxx = sxy = 0.0f;
    
    int pos = (cy-ts1)*xsize+(cx-ts1);
    if (vlen2s[pos] < 0)
        return -1;
    
    int ts = ts1 + ts2 + 1;
    if (vlen2s[pos] == 0) {
        
# pragma omp parallel shared ( ts, imageBW, tp) private ( j )
        for( j = 0; j < ts; ++j ) {
            ip = &imageBW[pos + j * xsize];
            ttp = &tp[j * ts];
# pragma omp parallel shared ( ts, ip, ttp) private ( i )
# pragma omp for reduction ( + : sx, +: sxx, +: sxy )
            
            for( i = 0; i < ts ; ++i ) {
                sx += ip[i];
                sxx += ip[i] * ip[i];
                sxy += ip[i] * ttp[i];
            }
        }
        
        vlen2 = sxx - sx*sx/(ts * ts);
        if( fabsf(vlen2) < 0.001 ) {
            vlen2s[pos] = -1;
            return -1;
        }
        
        vlen2 = sqrtf(vlen2);
        vlen2s[pos] = vlen2;
    } else {
# pragma omp parallel shared ( ts, imageBW, tp) private ( j )
        for( j = 0; j < ts; ++j ) {
            ip = &imageBW[pos + j * xsize];
            ttp = &tp[j * ts];
            
# pragma omp parallel shared ( ts, ip, ttp) private ( i )
# pragma omp for reduction ( + : sxy )
            for( i = 0; i < ts ; ++i ) {
                sxy += ip[i] * ttp[i];
            }
        }
        
        vlen2 = vlen2s[pos];
    }
    
    *sim = sxy / (vlen * vlen2);
    
    return 0;
}

static int make_template_origin( ARUint8 *imageBW, int xsize, int ysize,
                                int cx, int cy, int ts1, int ts2, float  sd_thresh,
                                float *template, float *vlen )
{
    ARUint8  *ip;
    float    *tp;
    float     vlen1, ave;
    int       i, j;
    
    if( cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize ) return -1;
    
    ave = 0.0f;
    for( j = -ts1; j <= ts2; j++ ) {
        ip = &imageBW[(cy+j)*xsize+(cx-ts1)];
        for( i = -ts1; i <= ts2 ; i++ ) ave += *(ip++);
    }
    ave /= (ts1+ts2+1)*(ts1+ts2+1);
    
    tp = template;
    vlen1 = 0.0f;
    for( j = -ts1; j <= ts2; j++ ) {
        ip = &imageBW[(cy+j)*xsize+(cx-ts1)];
        for( i = -ts1; i <= ts2 ; i++ ) {
            *tp = (float )(*(ip++)) - ave;
            vlen1 += *tp * *tp;
            tp++;
        }
    }
    
    if( vlen1 == 0.0f ) return -1;
    if( vlen1/((ts1+ts2+1)*(ts1+ts2+1)) < sd_thresh*sd_thresh ) return -1;
    
    *vlen = sqrtf(vlen1);
    
    return 0;
}

static int get_similarity_origin( ARUint8 *imageBW, int xsize, int ysize,
                                 float *template, float vlen, int ts1, int ts2,
                                 int cx, int cy, float  *sim)
{
    ARUint8   *ip;
    float     *tp;
    float     sx, sxx, sxy;
    float     vlen2;
    int       i, j;
    
    if( cy - ts1 < 0 || cy + ts2 >= ysize || cx - ts1 < 0 || cx + ts2 >= xsize ) return -1;
    
    tp = template;
    sx = sxx = sxy = 0.0f;
    for( j = -ts1; j <= ts2; j++ ) {
        ip = &imageBW[(cy+j)*xsize + (cx-ts1)];
        for( i = -ts1; i <= ts2 ; i++ ) {
            sx += *ip;
            sxx += *ip * *ip;
            sxy += *(ip++) * *(tp++);
        }
    }
    vlen2 = sxx - sx*sx/((ts1+ts2+1)*(ts1+ts2+1));
    if( vlen2 == 0.0f ) return -1;
    vlen2 = sqrtf(vlen2);
    
    *sim = sxy / (vlen * vlen2);
    
    return 0;
}