/**
 * @file WL_AT24CX.h
 * @author Muhammad Afif Ramadhan
 * @brief Implementation of AVR's high endurance EEPROM in ESP32 using AT24CX EEPROM
 * http://ww1.microchip.com/downloads/en/appnotes/doc2526.pdf
 *
 * Inherits AT24CX class from https://github.com/cyberp/AT24Cx
 *
 * @version 2
 * @date 2022-07-20
 *
 */

#include "AT24CX.h"

template <typename data_t>
struct wl_data_t {
    data_t data;
    uint32_t ptr;
    uint8_t crc;
} __attribute__((packed)); // packed to ensure sizeof returns correct struct size

/**
 * @brief EEPROM object based on AT24CX library
 *
 * @tparam data_t data type to be stored in eeprom
 */
template <class data_t>
class WL_AT24CX : public AT24CX {
   public:
    /**
     * @brief Construct a new WLAT24CX object
     *
     * @param index EEPROM index (A2 A1 A0)
     * @param pageSize EEPROM page size, from the manual
     * @param base_addr base eeprom address. get_end_addr() can be used to chain next class base address in ctor
     * @param num_of_data number of data to be stored in eeprom
     * @param wl_en enable/disable wear leveling
     * @param eeprom_size eeprom size, in bytes
     */
    WL_AT24CX(
        byte index,
        byte pageSize,
        uint32_t base_addr,
        uint32_t num_of_data,
        bool wl_en,
        uint32_t eeprom_size = 1 << 15)
        : AT24CX(index, pageSize)
    {
        this->base_addr   = base_addr;
        this->num_of_data = num_of_data;
        this->wl_enable   = wl_en;
        this->eeprom_size = eeprom_size;

        if (wl_enable)
            end_addr = base_addr + wl_data_size * num_of_data;
        else
            end_addr = base_addr + data_size * num_of_data;
        end_taddr  = this->num_of_data - 1; // taddr start from 0
        base_taddr = addr_to_taddr(base_addr);
    }

    /**
     * @brief Initialize by scanning eeprom for last pointer location
     *
     */
    void wl_init()
    {
        assert(wl_enable);

        wl_data_t<data_t> current, next;

        // Read from base address to end address
        for (uint32_t taddr = base_taddr; taddr <= end_taddr; taddr++) {
            current = wl_peek(taddr);
            next    = wl_peek(taddr + 1); // FIXME: might have bug if taddr == end_taddr

            // IF found break in pointer array
            if ((next.ptr - current.ptr != 1) || (next.ptr == pointer_max)) {
                uint32_t check_attempt = 0;
                // CRC validity check
                for (;;) {
                    // Wiped mem does not need for CRC check. Wiped mem when break found in taddr 0 AND next ptr is max
                    if ((taddr == 0) && (next.ptr == pointer_max)) {
                        taddr_current = 0; // circular taddr, HEAD taddr
                        taddr_last    = 0; // before HEAD taddr

                        wl_ptr_current = 0; // ptr to be written

                        goto out; // break valid, use taddr 0. Use goto to break out from nested loop
                    }

                    // check data validity
                    bool dataisvalid = isdatavalid(current);
                    if (!dataisvalid) {
                        ESP_LOGV("EEPROM WL", "CRC mismatch found!");
                        dataisvalid = isdatavalid(current);
                        // if data crc mismatch, go back one step in circular manner
                        if (taddr == 0)
                            taddr = end_taddr;
                        else
                            taddr--;

                        current = wl_peek(taddr);
                    } else {
                        taddr_current = (taddr + 1) % num_of_data; // HEAD taddr
                        taddr_last    = taddr;                     // Before HEAD taddr. Used to get last data

                        wl_ptr_current = current.ptr + 1; // ptr to be written, preincremented

                        goto out; // break valid, use taddr 0. Use goto to break out from nested loop
                    }

                    check_attempt++;
                    assert(check_attempt < num_of_data); // program must be error if this triggers
                }
            }
            // break not found, checking next taddr
        }
        // End taddr reached but no break found
        assert(0);
    out:
        ESP_LOGI("EEPROM WL", "Obtained taddr = %u, ptr %u", taddr_current, wl_ptr_current);
    }

    void wl_init2()
    {
        assert(wl_enable);

        wl_data_t<data_t> current, next;

        // Find pointer break
        uint32_t taddr;
        for (taddr = base_taddr; taddr <= end_taddr; taddr++) {
            current = wl_peek(taddr);
            next    = wl_peek(taddr + 1); // FIXME: might have bug if taddr == end_taddr?

            // found break in pointer array
            if ((next.ptr - current.ptr != 1) || (next.ptr == pointer_max))
                break;

            assert(taddr == end_taddr); // should not reach here

            // if (taddr == end_taddr) {
            //     taddr_current = 0;
            //     taddr_last    = 0;

            //     wl_ptr_current = 0;
            //     // TODO: raise status flag to inform function get last WL data to return 0 as well

            //     goto out;
            // }
        }

        // find data with correct crc
        uint32_t check_attempt = 0;
        bool dataisvalid;
        do {
            assert(check_attempt <= num_of_data); // should be found before reaches num of data

            dataisvalid = isdatavalid(current);
            if (!dataisvalid) {
                // if data crc mismatch, step backwards
                taddr_step(taddr, false);

                current = wl_peek(taddr);
            }
            check_attempt++;
        } while (!dataisvalid);

        // Data found, set as current taddr and set next pointer;
        taddr_last = taddr; // set taddr last to obtain last data stored

        taddr_step(taddr);
        taddr_current = taddr; // Preincrement taddr location

        wl_ptr_current = current.ptr + 1; // Preincrement next pointer to be written

        ESP_LOGI("EEPROM", "Obtained taddr = %u, ptr %u", taddr_current, wl_ptr_current);
    }

    /**
     * @brief push data to eeprom memory
     * contains assertion check for WL ENABLE!!
     *
     * @param data data to be put in eeprom
     */
    void wl_push(const data_t data)
    {
        assert(wl_enable);

        wl_data_t<data_t> buffer = { .data = data, .ptr = wl_ptr_current };

        uint32_t addr = taddr_to_addr(taddr_current);
        write(addr, reinterpret_cast<byte *>(&buffer), wl_data_size);

        wl_ptr_current++;
        taddr_last    = taddr_current;
        taddr_current = (taddr_current + 1) % num_of_data;
    }

    /**
     * @brief Write data to memory based on index taddr
     *
     * @param taddr array-like index
     * @param data data to be stored
     */
    void write_mem(const uint32_t taddr, const data_t data)
    {
        // enforce circular addressing on non-wl methods
        uint32_t addr = taddr_to_addr(taddr % num_of_data);
        data_t buffer = data;
        write(addr, reinterpret_cast<byte *>(&buffer), data_size);
    }

    /**
     * @brief Read data stored at given index
     *
     * @param taddr array-like index
     * @return data_t data stored at given index
     */
    data_t read_mem(const uint32_t taddr)
    {
        data_t out;
        read(taddr_to_addr(taddr % num_of_data), reinterpret_cast<byte *>(&out), data_size);
        return out;
    }

    /**
     * @brief Get end address (upper bounds) for this object
     * end address can be used as another wl_at24cx base address
     *
     * @return * uint32_t end address
     */
    uint32_t get_end_addr()
    {
        return end_addr;
    }

    /**
     * @brief WIPE data from eeprom, reset to 0xFF
     *  WARNING: wipe() does not limited by this object address bounds!!!!!
     *
     * @param size
     */
    void wipe(uint32_t size)
    {
        uint64_t max = -1;
        for (int i = 0; i < size; i += sizeof(uint64_t)) {
            ESP_LOGD("EEPROM", "Wiping process: %.2f", 100.0 * i / size);
            write(i, reinterpret_cast<byte *>(&max), sizeof(uint64_t));
        }
    }
    void wipe()
    {
        wipe(eeprom_size);
    }

    /**
     * @brief Get the last data stored by the wear-leveling algorithm
     *
     * @return data_t data last stored
     */
    data_t wl_get_last_data()
    {
        return wl_peek(taddr_last).data;
    }

    /**
     * @brief read data and pointer stored by the wear-leveling system
     *
     * @param taddr array-like indexing
     * @return wl_data_t<data_t> data struct containing data and pointer value
     */
    wl_data_t<data_t> wl_peek(uint32_t taddr)
    {
        wl_data_t<data_t> out;
        read(taddr_to_addr(taddr), reinterpret_cast<byte *>(&out), wl_data_size);
        // ESP_LOGD("EEPROM", "Obtained: index %d, addr %d, ptr %d, data %f",
        //  taddr, taddr_to_addr(taddr), out.ptr, out.data);
        return out;
    }

   private:
    uint32_t eeprom_size;

    uint32_t base_addr;
    uint32_t end_addr;

    uint32_t num_of_data;

    uint32_t base_taddr;
    uint32_t end_taddr;

    uint32_t data_size = sizeof(data_t);
    uint32_t taddr_current;

    uint32_t taddr_last;

    bool wl_enable;
    uint32_t wl_ptr_current;
    uint32_t wl_data_size = sizeof(wl_data_t<data_t>); // Size data struct plus pointer?

    uint32_t pointer_max = std::numeric_limits<uint32_t>::max();

    uint32_t taddr_to_addr(uint32_t taddr)
    {
        uint32_t addr;
        if (wl_enable)
            addr = taddr * wl_data_size;
        else
            addr = taddr * data_size;

        addr += base_addr;
        return addr;
    }

    uint32_t addr_to_taddr(uint32_t addr)
    {
        uint32_t taddr;
        if (wl_enable)
            taddr = (addr - base_addr) / wl_data_size;
        else
            taddr = (addr - base_addr) / data_size;
        return taddr;
    }

    /**
     * @brief function to calculate CRC
     *
     * @param data data type
     * @return uint8_t 8-bit crc
     */
    uint8_t calc_crc(data_t data)
    {
        uint8_t output = 0;
        data_t *dataptr;

        *dataptr = data;

        for (size_t i = 0; i < sizeof(data_t); i++) {
            output ^= dataptr[i];
        }

        return output;
    }

    /**
     * @brief function to change taddr forward and backwards in circular manner
     *
     * @param taddr reference to taddr
     * @param forward defaults to true, set false to step backwards
     */
    void taddr_step(uint32_t &taddr, bool forward = true)
    {
        if (forward) {
            taddr = (taddr + 1) % num_of_data;
        } else { // step backwards
            // if reaches end of taddr
            if (taddr == 0)
                taddr = end_taddr;
            else
                taddr--;
        }
    }

    /**
     * @brief function to check data validity based on crc
     *
     * @param input WL data struct
     * @return true means data and crc is valid.
     * @return false means crc mismatch.
     */
    bool isdatavalid(wl_data_t<data_t> input)
    {
        bool isvalid = false;
        if (input.crc == calc_crc(input.data))
            isvalid = true;

        return isvalid;
    }
};
