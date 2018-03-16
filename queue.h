#ifndef QUEUE_DOT_H
#define QUEUE_DOT_H

typedef struct link_struct
{
    struct link_struct *next;
}link_t;

typedef struct queue_struct
{
    struct link_struct *front_ptr;
    struct link_struct *rear_ptr;
    int count;
}queue_t;

void  queue_item_init(queue_t *q);
void  queue_item_push(queue_t* q, void* pitem);
void* queue_item_pop(queue_t* q);
int   queue_item_count(queue_t* q);

#endif
