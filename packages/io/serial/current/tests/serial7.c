#include <pkgconf/system.h>

#include <cyg/infra/testcase.h>         // test macros
#include <cyg/infra/cyg_ass.h>          // assertion macros


#include <pkgconf/kernel.h>


#include <cyg/hal/hal_arch.h>           // CYGNUM_HAL_STACK_SIZE_TYPICAL
#include <cyg/kernel/kapi.h>

#include "ser_test_protocol.inl"


int main(void)
{
 cyg_io_handle_t handle;
 Cyg_ErrNo err,err_read;
 cyg_uint32 len_read = 0;
 char read_string[50];
 const char test_string[] = "Serial example with API function cyg_io_write is working correctly!\n";
// char read_string[];
 cyg_uint32 len = strlen(test_string);

 printf("Starting serial example\n");
 err = cyg_io_lookup( "/dev/ttydiag", &handle );
 if (ENOERR == err) 
 {
  printf("Found /dev/ttydiag. Writing string....\n");
//  while(1)
  err = cyg_io_write( handle, test_string, &len );
 } 
 if (ENOERR == err) 
 {
  printf("Writing operation finished succesful.\n");
 }
// while(len_read < 8)
// {
  cyg_io_read(handle, read_string, 8);
  
  
//  if (ENOERR == err_read)
//  {
   printf(read_string);//read_string);
//   printf("\n%d",len_read);
//  }
// }
}