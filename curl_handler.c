#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <pthread.h>
#include <stdbool.h>
#include <sys/syscall.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <curl/curl.h>
#include "engienl.h"
#include "typedefs.h"
#include "queue.h"
#include "curl_handler.h"

#define SUBMIT_READINGS_FILE      ".submitReadings.json"
#define MAX_POWER_PAYLOAD 32


static char powerURL[128];
static char readingsURL[128];
static queue_t queue;
static pthread_mutex_t  mutex;
static CURL* curl;

//
// Private function
//
static void  _send_text_plain(const char* payload);
static void  _send_application_json(const char* payload, int length);


void curl_sendPowerToDeliver(uint16_t power)
{
    queue_item_t* pdata;
    char *payload;

    pdata = malloc(sizeof(queue_item_t));
    payload = malloc(MAX_POWER_PAYLOAD);

    if (pdata && payload)
    {
        pdata->link.next = NULL;  // !!! Don't forget to NULL next item before queue.
        pdata->payload = payload;
        if ( power & 0x8000 )     // is most significant bit set
        {
            power = ((~power) + 1 );
            sprintf(pdata->payload,"/powerToDeliver/-%d", power);
        }
        else
        {
            sprintf(pdata->payload,"/powerToDeliver/%d", power);
        }
        pdata->type = CURL_PLAIN_TEXT;
        pthread_mutex_lock(&mutex);
        queue_item_push(&queue, pdata);
        pthread_mutex_unlock(&mutex);
    }
}

void curl_sendReadings(const char* readings, int length)
{
    queue_item_t* pdata;
    char *payload;

    pdata = malloc(sizeof(queue_item_t));
    payload = malloc(length);

    if (pdata && payload)
    {
        pdata->link.next = NULL;  // !!! Don't forget to NULL next item before queue.
        pdata->payload = payload;
        pdata->length = length;
        memcpy(payload, readings, length);
        pdata->type = CURL_APPLICATION_JSON;
        pthread_mutex_lock(&mutex);
        queue_item_push(&queue, pdata);
        pthread_mutex_unlock(&mutex);
    }
}


void _send_text_plain(const char* payload)
{
	const int payloadLength = 128;
    struct curl_slist *headers = NULL;
    curl = curl_easy_init();

    if (curl)
    {
        char buf[payloadLength];
        sprintf(buf, "%s%s", powerURL, payload);
		curl_slist_append(headers, "Content-Type: text/plain");
		curl_slist_append(headers, "charsets: utf-8");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_URL, buf);
        curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
        curl_easy_perform(curl);
        curl_slist_free_all(headers);
    }
    curl_easy_cleanup(curl);
}

static size_t read_callback(void *ptr, size_t size, size_t nmemb, void *stream)
{
  size_t retcode;
  curl_off_t nread;

  retcode = fread(ptr, size, nmemb, stream);
  nread = (curl_off_t)retcode;
  return retcode;
}

void _send_application_json(const char* payload, int length)
{
	FILE *fp;
	struct stat file_info;
    const char* file = SUBMIT_READINGS_FILE;
    fp = fopen ( file, "wb");
    if ( fp )
    {
    	fwrite(payload, length, 1, fp);
        fclose(fp);
    }
	stat(file, &file_info);
	fp = fopen(file, "rb");
	curl = curl_easy_init();
    if (curl)
    {
	    curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);
	    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
	    curl_easy_setopt(curl, CURLOPT_PUT, 1L);
	    curl_easy_setopt(curl, CURLOPT_URL, readingsURL);
	    curl_easy_setopt(curl, CURLOPT_READDATA, fp);
	    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE,
	                     (curl_off_t)file_info.st_size);

		curl_easy_perform(curl);
    }
    fclose(fp); /* close the local file */
    curl_easy_cleanup(curl);
}


void *curl_handler( void *ptr )
{
    int count = 0;
    curl_thread_param_t* param = (curl_thread_param_t*) ptr;
    uint8_t *terminate = param->terminate;
    strcpy(powerURL, param->powerToDeliverURL);
    strcpy(readingsURL, param->submitReadingsURL);
    free(param);

    pthread_mutex_init(&mutex, NULL);
    queue_item_init(&queue);

    while (*terminate == false)
    {
        count = queue_item_count(&queue);
        if (count )
        {
            queue_item_t *pdata;
            pthread_mutex_lock(&mutex);
            pdata = (queue_item_t*) queue_item_pop(&queue);
            pthread_mutex_unlock(&mutex);
            if ( pdata->type == CURL_PLAIN_TEXT )
            {
                _send_text_plain(pdata->payload);
            }
            else
            {
                _send_application_json(pdata->payload, pdata->length);
            }
            free(pdata->payload);
            free(pdata);
        }
        sleep(1);
    }

    return 0;
}
