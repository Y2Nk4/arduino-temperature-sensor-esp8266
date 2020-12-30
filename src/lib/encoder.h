#ifndef _ENCODER_h
	#define _ENCODER_h

    bool TemperatureRecordEncodeCb(pb_ostream_t* stream, const pb_field_t* field,
        void* const* arg)
    {
        Serial.printf("called, field->tag=%d field->type=%d", field->tag, field->type);

        for (int i = 0; i < 4; i++)
        {
            if (!pb_encode_tag_for_field(stream, field))
            {
                Serial.println("encode failed");
                return false;
            }

            TemperatureRecord tempRecord = TemperatureRecord_init_zero;

            if (!pb_encode_submessage(stream, TemperatureRecord_fields, &tempRecord))
            {
                Serial.println("encode failed");
                return false;
            }
        }

        return true;
    }

    //bool RunRecord_callback(pb_istream_t* istream, pb_ostream_t* ostream, const pb_field_iter_t* field)
    //{
    //    PB_UNUSED(istream);
    //    if (ostream != NULL && field->tag == RunRecord_temperature_record_tag)
    //    {
    //        // field->pData is the raw buffer
    //        // create a sub-message instance here to prepare to encode

    //        // DIR* dir = *(DIR**)field->pData;
    //        // struct dirent* file;
    //        // FileInfo fileinfo = {};

    //        // loop for all the submessages
    //        // while ((file = readdir(dir)) != NULL)
    //        {
    //            // encode each sub-message into the encoding stream
    //            // fileinfo.inode = file->d_ino;

    //            /* This encodes the header for the field, based on the constant info
    //            * from pb_field_t. */
    //            if (!pb_encode_tag_for_field(ostream, field))
    //                return false;

    //            /* This encodes the data for the field, based on our FileInfo structure. */
    //            if (!pb_encode_submessage(ostream, TemperatureRecord_fields, &fileinfo))
    //                return false;
    //        }
    //    }

    //    return true;
    //}

    bool RunRecord_callback(pb_istream_t* istream, pb_ostream_t* ostream, const pb_field_iter_t* field)
    {
        PB_UNUSED(ostream);
        if (istream != NULL && field->tag == RunRecord_temperature_record_tag)
        {
            TemperatureRecord tmpRecord = {};

            if (!pb_decode(istream, TemperatureRecord_fields, &tmpRecord))
                return false;

            Serial.print("decoded:");
            Serial.println(tmpRecord.temperature_value);
        }

        return true;
    }
#endif