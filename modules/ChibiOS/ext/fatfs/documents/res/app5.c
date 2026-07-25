/*----------------------------------------------------------------------/
/ Test if the file is contiguous                                        /
/----------------------------------------------------------------------*/

FRESULT test_contiguous_file (
    FIL* fp,    /* [IN]  Open file object to be checked */
    int* cont   /* [OUT] 1:Contiguous, 0:Fragmented or zero-length */
)
{
    DWORD clst, clsz, step;
    FSIZE_t fsz;
    FRESULT fr;


    *cont = 0;
    fr = f_lseek(fp, 0);            /* Validates and prepares the file */
    if (fr != FR_OK) return fr;

#if FF_MAX_SS == FF_MIN_SS
    clsz = (DWORD)fp->obj.fs->csize * FF_MAX_SS;    /* Cluster size */
#else
    clsz = (DWORD)fp->obj.fs->csize * fp->obj.fs->ssize;
#endif
    fsz = f_size(fp);
    if (fsz > 0) {
        clst = fp->obj.sclust - 1;  /* A cluster leading the first cluster for first test */
        while (fsz) {
            step = (fsz >= clsz) ? clsz : (DWORD)fsz;
            fr = f_lseek(fp, f_tell(fp) + step);    /* Advances file pointer a cluster */
            if (fr != FR_OK) return fr;
            if (clst + 1 != fp->clust) break;       /* Is not the cluster next to previous one? */
            clst = fp->clust; fsz -= step;          /* Get current cluster for next test */
        }
        if (fsz == 0) *cont = 1;    /* All done without fail? */
    }

    return FR_OK;
}
