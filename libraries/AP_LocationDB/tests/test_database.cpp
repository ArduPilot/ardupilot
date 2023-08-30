#include <AP_gtest.h>

/*
  tests for AP_LocationDB
 */

#include <AP_LocationDB/AP_LocationDB.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX

TEST(KeyValidation, key_validation) {
    uint32_t test_key = AP_LocationDB::construct_key_mavlink(1, 255, 33);
    EXPECT_TRUE(AP_LocationDB::is_valid_key(test_key));

    test_key = AP_LocationDB::construct_key_mavlink(1, 255, 144);
    EXPECT_TRUE(AP_LocationDB::is_valid_key(test_key));

    test_key = AP_LocationDB::construct_key_mavlink(0, 255, 144); // zero sysid not allowed
    EXPECT_FALSE(AP_LocationDB::is_valid_key(test_key));

    test_key = AP_LocationDB::construct_key_mavlink(1, 255, 150); // message id not supported
    EXPECT_FALSE(AP_LocationDB::is_valid_key(test_key));

    test_key = AP_LocationDB::construct_key_adsb(1234);
    EXPECT_TRUE(AP_LocationDB::is_valid_key(test_key));

    test_key = AP_LocationDB::construct_key_scripting(4321);
    EXPECT_TRUE(AP_LocationDB::is_valid_key(test_key));

    test_key = ((255 << 24) | 1234); // invalid domain
    EXPECT_FALSE(AP_LocationDB::is_valid_key(test_key));
}

TEST(KeyConstruction, key_construction) {
    uint8_t msgid = 144;
    uint8_t compid = 197;
    uint8_t sysid = 24;
    uint32_t key = AP_LocationDB::construct_key_mavlink(sysid, compid, msgid);

    EXPECT_EQ((AP_LocationDB::KeyDomain)((key >> 24) & 255U), AP_LocationDB::KeyDomain::MAVLINK);
    EXPECT_EQ((key >> 16) & 255U, sysid);
    EXPECT_EQ((key >> 8) & 255U, compid);
    EXPECT_EQ((key) & 255U, AP_LocationDB::short_mav_msg_id(msgid));

    uint32_t icao = (1U << 27) - 53U;
    key = AP_LocationDB::construct_key_adsb(icao);
    EXPECT_EQ((AP_LocationDB::KeyDomain)((key >> 24) & 255U), AP_LocationDB::KeyDomain::ADSB);
    EXPECT_NE(((1U << 24) - 1U) & key, icao);   // icao number must be of 24 bits. Bits higher than the 24th bit are zeroed

    icao =  (1U << 24) - 144U;
    key = AP_LocationDB::construct_key_adsb(icao);
    EXPECT_EQ((AP_LocationDB::KeyDomain)((key >> 24) & 255U), AP_LocationDB::KeyDomain::ADSB);
    EXPECT_EQ(((1U << 24) - 1U) & key, icao);

    uint32_t subkey =  (1U << 24) - 144U;
    key = AP_LocationDB::construct_key_scripting(subkey);
    EXPECT_EQ((AP_LocationDB::KeyDomain)((key >> 24) & 255U), AP_LocationDB::KeyDomain::SCRIPTING);
    EXPECT_EQ(((1U << 24) - 1U) & key, subkey);
}

static AP_LocationDB_Item create_db_item(uint8_t fields_to_populate, uint32_t key) {
    uint32_t timestamp = AP_HAL::millis();
    uint32_t item_key = AP_LocationDB::construct_key_scripting(key);

    // assign these fields a value as a function of key
    float field_value = key + 1;
    Vector3f pos {field_value, field_value, field_value};
    field_value = key + 10;
    Vector3f vel {field_value, field_value, field_value};
    field_value = key + 100;
    Vector3f acc {field_value, field_value, field_value};
    field_value = key + 1000;
    float heading = field_value;
    field_value = key + 10000;
    float radius = field_value;
    AP_LocationDB_Item item(item_key, timestamp, pos, vel, acc, heading, radius, fields_to_populate);

    return item;
}

TEST(ItemFields, item_fields) {
    for (uint8_t i = 0; i < (1 << 5); i++) {
        AP_LocationDB_Item item = create_db_item(i, 2500);
        Vector3f dummy_vector3f;
        float dummy_float;

        // Checking if we are able to retrive a populated field and vice versa
        // Using XOR (^) operator to match the received output of getters with the expected output
        // XOR returns false when both the operands have same value, i.e., either both of them are true or both are false
        EXPECT_FALSE(item.get_pos_cm_NEU(dummy_vector3f) ^ ((i & (uint8_t)AP_LocationDB_Item::DataField::POS) != 0));
        EXPECT_FALSE(item.get_vel_cm_NEU(dummy_vector3f) ^ ((i & (uint8_t)AP_LocationDB_Item::DataField::VEL) != 0));
        EXPECT_FALSE(item.get_acc_cm_NEU(dummy_vector3f) ^ ((i & (uint8_t)AP_LocationDB_Item::DataField::ACC) != 0));
        EXPECT_FALSE(item.get_heading_cdeg(dummy_float) ^ ((i & (uint8_t)AP_LocationDB_Item::DataField::HEADING) != 0));
        EXPECT_FALSE(item.get_radius_cm(dummy_float) ^ ((i & (uint8_t)AP_LocationDB_Item::DataField::RADIUS) != 0));
    }
}

static void check_equal(AP_LocationDB_Item& item1, AP_LocationDB_Item& item2) {
    uint32_t timestamp_ms1 = item1.get_timestamp_ms(), timestamp_ms2 = item2.get_timestamp_ms();
    Vector3f pos1, pos2;
    Vector3f vel1, vel2;
    Vector3f acc1, acc2;
    float heading1, heading2;
    float radius1, radius2;

    EXPECT_TRUE(item1.get_pos_cm_NEU(pos1));
    EXPECT_TRUE(item1.get_vel_cm_NEU(vel1));
    EXPECT_TRUE(item1.get_acc_cm_NEU(acc1));
    EXPECT_TRUE(item1.get_heading_cdeg(heading1));
    EXPECT_TRUE(item1.get_radius_cm(radius1));
    
    EXPECT_TRUE(item2.get_pos_cm_NEU(pos2));
    EXPECT_TRUE(item2.get_vel_cm_NEU(vel2));
    EXPECT_TRUE(item1.get_acc_cm_NEU(acc2));
    EXPECT_TRUE(item2.get_heading_cdeg(heading2));
    EXPECT_TRUE(item2.get_radius_cm(radius2));
    
    EXPECT_EQ(timestamp_ms1, timestamp_ms2);
    EXPECT_EQ(pos1, pos2);
    EXPECT_EQ(vel1, vel2);
    EXPECT_EQ(acc1, acc2);
    EXPECT_TRUE(is_equal(heading1, heading2));
    EXPECT_TRUE(is_equal(radius1, radius2));

    return;
}

AP_LocationDB db;

TEST(Insertion, insert) {
    db.init();
    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == 0);

    AP_LocationDB_Item arr[db.capacity()];

    for (int i=0; i<db.capacity(); i++) {
        AP_LocationDB_Item item = create_db_item((1 << 5) - 1, i); // data item with all fields populated
        arr[i] = item;
    }

    for (int i=0; i<db.capacity(); i++) {
        db.add_item(arr[i]);
    }

    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == db.capacity());
    EXPECT_TRUE(db.is_full());

    // check consistency
    for (int i=0; i<db.capacity(); i++) {
        AP_LocationDB_Item item;
        EXPECT_TRUE(db.get_item(arr[i].get_key(), item));
        check_equal(item, arr[i]);
    }
}

TEST(Deletion, deletion) {
    db.init();
    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == 0);

    int item_count = 99;
    uint32_t keys[item_count];

    for (int i=0; i<item_count; i++) {
        AP_LocationDB_Item item = create_db_item((1 << 5) - 1, i); // data item with all fields populated
        keys[i] = item.get_key();
        EXPECT_TRUE(db.add_item(item));
    }

    EXPECT_TRUE(db.size() == item_count);

    // delete other elements
    while(item_count > 0) {
        EXPECT_TRUE(db.remove_item(keys[0]));
        keys[0] = keys[item_count - 1];
        item_count -= 1;
        EXPECT_TRUE(db.healthy());
        EXPECT_TRUE(db.size() == item_count);
    }

    EXPECT_TRUE(db.size() == 0);
}

TEST(Updation, updation) {
    db.init();
    EXPECT_TRUE(db.healthy());
    EXPECT_TRUE(db.size() == 0);

    int item_count = 99;
    uint32_t keys[item_count];

    for (int i=0; i<item_count; i++) {
        AP_LocationDB_Item item = create_db_item((1 << 5) - 1, i); // data item with all fields populated
        keys[i] = item.get_key();
        EXPECT_TRUE(db.add_item(item));
    }

    for (int i=0; i<100; i++) {
        int update_index = i % db.size();
        AP_LocationDB_Item new_item = create_db_item((1 << 5) - 1, i * 100); // data item with all fields populated
        EXPECT_TRUE(db.update_item(keys[update_index], new_item));
        keys[update_index] = new_item.get_key();

        // check consistency
        AP_LocationDB_Item retrieved_item;
        EXPECT_TRUE(db.get_item(keys[update_index], retrieved_item));
        check_equal(new_item, retrieved_item);
    }
}

AP_GTEST_MAIN()

#endif // HAL_SITL or HAL_LINUX
