/*
 * File_Handling_RTOS.h
 *
 *  Created on: 30-April-2020
 *      Author: Controllerstech
 */

#ifndef FILE_HANDLING_RTOS_H_
#define FILE_HANDLING_RTOS_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "fatfs_sd.h"


/* mounts the sd card
 * @param path : mount location
 */
void Mount_SD (const TCHAR* path);

/* unmounts the sd card
 * @param path : mount location
 */
void Unmount_SD (const TCHAR* path);

/* Start node to be scanned (***also used as work area***)
 * @param pat : directory path for scanning
 */
FRESULT Scan_SD (char* pat);

/* Only supports removing files from home directory. Directory remover to be added soon */
FRESULT Format_SD (void);

/* write the data to the file
 * @param name : path to the file
 * @param data : null terminated string to write*/
FRESULT Write_File (char *name, char *data);

/* read data from the file
 * @param name : is the path to the file*/
FRESULT Read_File (char *name);

/* creates the file, if it does not exists
 * @param name : is the path to the file*/
FRESULT Create_File (char *name);

/* Removes the file from the sd card
 * @param name : is the path to the file*/
FRESULT Remove_File (char *name);

/* creates a directory
 * @param name: is the path to the directory
 */
FRESULT Create_Dir (char *name);

/* checks the free space in the sd card*/
void Check_SD_Space (void);

/* updates the file. write pointer is set to the end of the file
 * @param name : path to the file
 * @param data : data to append
 */
FRESULT Update_File (char *name, char *data);




#endif /* FILE_HANDLING_RTOS_H_ */
