#pragma once

// table maps between string names and pointer to element
// Function pointer that matches signature of generated topics
typedef bool (*Generic_serialize_topic_fn_t)(struct ucdrBuffer*, const void*);
typedef bool (*Generic_deserialize_topic_fn_t)(struct ucdrBuffer*, void*);
typedef uint32_t (*Generic_size_of_topic_fn_t)(struct ucdrBuffer*, uint32_t);
