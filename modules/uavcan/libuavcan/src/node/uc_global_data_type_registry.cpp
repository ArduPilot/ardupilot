/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/debug.hpp>
#include <cassert>
#include <cstdlib>

namespace uavcan
{

GlobalDataTypeRegistry::List* GlobalDataTypeRegistry::selectList(DataTypeKind kind) const
{
    if (kind == DataTypeKindMessage)
    {
        return &msgs_;
    }
    else if (kind == DataTypeKindService)
    {
        return &srvs_;
    }
    else
    {
        UAVCAN_ASSERT(0);
        return UAVCAN_NULLPTR;
    }
}

GlobalDataTypeRegistry::RegistrationResult GlobalDataTypeRegistry::remove(Entry* dtd)
{
    if (!dtd)
    {
        UAVCAN_ASSERT(0);
        return RegistrationResultInvalidParams;
    }
    if (isFrozen())
    {
        return RegistrationResultFrozen;
    }

    List* list = selectList(dtd->descriptor.getKind());
    if (!list)
    {
        return RegistrationResultInvalidParams;
    }

    list->remove(dtd);       // If this call came from regist<>(), that would be enough
    Entry* p = list->get();  // But anyway
    while (p)
    {
        Entry* const next = p->getNextListNode();
        if (p->descriptor.match(dtd->descriptor.getKind(), dtd->descriptor.getFullName()))
        {
            list->remove(p);
        }
        p = next;
    }
    return RegistrationResultOk;
}

GlobalDataTypeRegistry::RegistrationResult GlobalDataTypeRegistry::registImpl(Entry* dtd)
{
    if (!dtd || !dtd->descriptor.isValid())
    {
        UAVCAN_ASSERT(0);
        return RegistrationResultInvalidParams;
    }
    if (isFrozen())
    {
        return RegistrationResultFrozen;
    }

    List* list = selectList(dtd->descriptor.getKind());
    if (!list)
    {
        return RegistrationResultInvalidParams;
    }

    {   // Collision check
        Entry* p = list->get();
        while (p)
        {
            if (p->descriptor.getID() == dtd->descriptor.getID()) // ID collision
            {
                return RegistrationResultCollision;
            }
            if (!std::strncmp(p->descriptor.getFullName(), dtd->descriptor.getFullName(),
                              DataTypeDescriptor::MaxFullNameLen))                        // Name collision
            {
                return RegistrationResultCollision;
            }
            p = p->getNextListNode();
        }
    }
#if UAVCAN_DEBUG
    const unsigned len_before = list->getLength();
#endif
    list->insertBefore(dtd, EntryInsertionComparator(dtd));

#if UAVCAN_DEBUG
    {   // List integrity check
        const unsigned len_after = list->getLength();
        if ((len_before + 1) != len_after)
        {
            UAVCAN_ASSERT(0);
            std::abort();
        }
    }
    {   // Order check
        Entry* p = list->get();
        int id = -1;
        while (p)
        {
            if (id >= p->descriptor.getID().get())
            {
                UAVCAN_ASSERT(0);
                std::abort();
            }
            id = p->descriptor.getID().get();
            p = p->getNextListNode();
        }
    }
#endif
    return RegistrationResultOk;
}

GlobalDataTypeRegistry& GlobalDataTypeRegistry::instance()
{
    static GlobalDataTypeRegistry singleton;
    return singleton;
}

void GlobalDataTypeRegistry::freeze()
{
    if (!frozen_)
    {
        frozen_ = true;
        UAVCAN_TRACE("GlobalDataTypeRegistry", "Frozen; num msgs: %u, num srvs: %u",
                     getNumMessageTypes(), getNumServiceTypes());
    }
}

const DataTypeDescriptor* GlobalDataTypeRegistry::find(const char* name) const
{
    const DataTypeDescriptor* desc = find(DataTypeKindMessage, name);
    if (desc == UAVCAN_NULLPTR)
    {
        desc = find(DataTypeKindService, name);
    }
    return desc;
}

const DataTypeDescriptor* GlobalDataTypeRegistry::find(DataTypeKind kind, const char* name) const
{
    if (!name)
    {
        UAVCAN_ASSERT(0);
        return UAVCAN_NULLPTR;
    }
    const List* list = selectList(kind);
    if (!list)
    {
        UAVCAN_ASSERT(0);
        return UAVCAN_NULLPTR;
    }
    Entry* p = list->get();
    while (p)
    {
        if (p->descriptor.match(kind, name))
        {
            return &p->descriptor;
        }
        p = p->getNextListNode();
    }
    return UAVCAN_NULLPTR;
}

const DataTypeDescriptor* GlobalDataTypeRegistry::find(DataTypeKind kind, DataTypeID dtid) const
{
    const List* list = selectList(kind);
    if (!list)
    {
        UAVCAN_ASSERT(0);
        return UAVCAN_NULLPTR;
    }
    Entry* p = list->get();
    while (p)
    {
        if (p->descriptor.match(kind, dtid))
        {
            return &p->descriptor;
        }
        p = p->getNextListNode();
    }
    return UAVCAN_NULLPTR;
}

}
