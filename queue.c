#include <stdio.h>
#include "queue.h"


void queue_item_init(queue_t* q)
{
    q->front_ptr = NULL;
    q->rear_ptr = NULL;
    q->count = 0;
}

void queue_item_push(queue_t* q, void* pitem)
{
    link_t *item = (link_t*)pitem;
    //printf("%s - %p\n", __PRETTY_FUNCTION__, item);

    if (q->front_ptr == NULL)
    {
        q->front_ptr = item;
    }
    else
    {
        q->rear_ptr->next = item;
    }
    q->rear_ptr = item;
    q->count++;
}

void* queue_item_pop(queue_t* q)
{
    link_t *item =  NULL;

    if (q->front_ptr)
    {
        item = q->front_ptr;
        q->front_ptr = q->front_ptr->next;
        q->count--;
    }
    //printf("%s - %p\n", __PRETTY_FUNCTION__, item);

    return item;
}

int queue_item_count(queue_t* q)
{
    return q->count;
}
